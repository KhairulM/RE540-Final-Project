#!/usr/bin/env python3
import rospy
import os
import json
import time
import base64
from openai import OpenAI
from pydantic import BaseModel
from typing import Literal, Optional
from pilot.srv import ChatCompletion, ChatCompletionResponse

# History management constants
MAX_HISTORY = 20        # Maximum conversation turns to keep in memory
SUMMARY_TRIGGER = 10    # Trigger summary after this many turns

# Structured output models
class StoreIdentification(BaseModel):
    """Structured output for store type identification."""
    store_type: Literal["cafe", "pharmacy", "convenience store", "restaurant", "unknown"]
    confidence: Literal["high", "medium", "low"]
    reasoning: str


class LLMAgentNode:
    def __init__(self):
        rospy.init_node('llm_agent_node')
        
        # Get OpenAI API key from environment
        self.api_key = os.environ.get('OPENAI_API_KEY')
        if not self.api_key:
            rospy.logerr("[LLMAgent] OPENAI_API_KEY not found in environment variables!")
            rospy.logerr("[LLMAgent] Please set it with: export OPENAI_API_KEY='your-key-here'")
            raise ValueError("OPENAI_API_KEY environment variable not set")
        
        # Initialize OpenAI client
        try:
            self.client = OpenAI(api_key=self.api_key)
            rospy.loginfo("[LLMAgent] OpenAI client initialized successfully")
        except Exception as e:
            rospy.logerr("[LLMAgent] Failed to initialize OpenAI client: %s", str(e))
            raise
        
        # Get parameters
        self.model = rospy.get_param("~model", "gpt-4o-mini")
        self.default_temperature = rospy.get_param("~default_temperature", 0.7)
        self.default_max_tokens = rospy.get_param("~default_max_tokens", 500)
        self.default_system_message = rospy.get_param(
            "~default_system_message", 
            "You are a helpful AI assistant for a mobile robot with long-term memory."
        )
        self.enable_history = rospy.get_param("~enable_history", True)
        self.max_history = rospy.get_param("~max_history", MAX_HISTORY)
        self.summary_trigger = rospy.get_param("~summary_trigger", SUMMARY_TRIGGER)
        self.use_structured_output = rospy.get_param("~use_structured_output", False)
        
        # Initialize history management
        self.turn_history = []
        self.summary_memory = ""
        
        if self.enable_history:
            self._setup_history()
        else:
            rospy.loginfo("[LLMAgent] History disabled")
            self.history_file = None
            self.summary_path = None
        
        # Create service
        self.chat_service = rospy.Service(
            '/llm_chat', 
            ChatCompletion, 
            self.handle_chat_request
        )
        
        rospy.loginfo("[LLMAgent] LLM Agent Node initialized.")
        rospy.loginfo("[LLMAgent] Model: %s", self.model)
        rospy.loginfo("[LLMAgent] History enabled: %s", self.enable_history)
        if self.enable_history:
            rospy.loginfo("[LLMAgent] History file: %s", self.history_file)
            rospy.loginfo("[LLMAgent] Summary path: %s", self.summary_path)
        rospy.loginfo("[LLMAgent] Service available at: /llm_chat")
    
    def _setup_history(self):
        """Setup history directory and files."""
        # Create history directory
        pkg_path = os.path.dirname(os.path.abspath(__file__))
        self.history_dir = os.path.join(pkg_path, "history")
        os.makedirs(self.history_dir, exist_ok=True)
        
        # Create session-specific history file
        timestamp = int(time.time())
        self.history_file = os.path.join(
            self.history_dir,
            f"session_{timestamp}.jsonl"
        )
        
        # Load previous summary if exists
        self.summary_path = os.path.join(self.history_dir, "summary.json")
        if os.path.exists(self.summary_path):
            try:
                with open(self.summary_path, 'r') as f:
                    data = json.load(f)
                    self.summary_memory = data.get("summary", "")
                    if self.summary_memory:
                        rospy.loginfo("[LLMAgent] Loaded previous memory summary (%d chars)", 
                                     len(self.summary_memory))
            except Exception as e:
                rospy.logwarn("[LLMAgent] Could not load summary: %s", str(e))
                self.summary_memory = ""
        
        rospy.loginfo("[LLMAgent] History directory: %s", self.history_dir)
    
    def _encode_image(self, image_path):
        """Encode image to base64 string."""
        try:
            with open(image_path, "rb") as image_file:
                return base64.b64encode(image_file.read()).decode('utf-8')
        except Exception as e:
            rospy.logerr("[LLMAgent] Failed to encode image: %s", str(e))
            raise
    
    def _read_text_file(self, file_path):
        """Read text file content."""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                return f.read()
        except Exception as e:
            rospy.logerr("[LLMAgent] Failed to read text file: %s", str(e))
            raise
    
    def _get_image_mime_type(self, file_path):
        """Determine image MIME type from file extension."""
        ext = os.path.splitext(file_path)[1].lower()
        mime_types = {
            '.jpg': 'image/jpeg',
            '.jpeg': 'image/jpeg',
            '.png': 'image/png',
            '.gif': 'image/gif',
            '.webp': 'image/webp'
        }
        return mime_types.get(ext, 'image/jpeg')
    
    def _save_entry(self, role, content):
        """Save a conversation entry to history."""
        if not self.enable_history:
            return
            
        entry = {"role": role, "content": content, "timestamp": time.time()}
        self.turn_history.append(entry)
        
        # Keep only recent history
        if len(self.turn_history) > self.max_history:
            self.turn_history = self.turn_history[-self.max_history:]
        
        # Write to file
        try:
            with open(self.history_file, 'a') as f:
                f.write(json.dumps(entry) + "\n")
        except Exception as e:
            rospy.logwarn("[LLMAgent] Failed to write history: %s", str(e))
    
    def _build_messages(self, user_prompt, system_message, image_content=None, text_file_content=None):
        """Build message list with history, summary, and multimodal content."""
        messages = [{"role": "system", "content": system_message}]
        
        # Add summary memory if available
        if self.enable_history and self.summary_memory:
            messages.append({
                "role": "system",
                "content": f"[LONG-TERM MEMORY SUMMARY]\n{self.summary_memory}"
            })
        
        # Add recent conversation history
        if self.enable_history:
            for entry in self.turn_history:
                messages.append({
                    "role": entry["role"],
                    "content": entry["content"]
                })
        
        # Build user message with multimodal content
        user_message = {"role": "user", "content": []}
        
        # Add text file content first if provided
        if text_file_content:
            user_message["content"].append({
                "type": "text",
                "text": f"[ATTACHED TEXT FILE CONTENT]\n{text_file_content}\n\n"
            })
        
        # Add main prompt
        user_message["content"].append({
            "type": "text",
            "text": user_prompt
        })
        
        # Add image if provided
        if image_content:
            user_message["content"].append(image_content)
        
        # If only text (no multimodal), simplify the structure
        if not image_content and not text_file_content:
            user_message["content"] = user_prompt
        
        messages.append(user_message)
        
        return messages
    
    def _update_summary(self):
        """Update memory summary when history gets long."""
        if not self.enable_history:
            return
            
        if len(self.turn_history) < self.summary_trigger:
            return
        
        try:
            rospy.loginfo("[LLMAgent] Updating memory summary (%d turns)...", 
                         len(self.turn_history))
            
            # Create summary prompt
            history_text = json.dumps(
                [{"role": e["role"], "content": e["content"]} 
                 for e in self.turn_history],
                indent=2
            )
            
            summary_messages = [
                {
                    "role": "system",
                    "content": "You are a memory summarizer. Create a concise summary of the conversation history that preserves key information, decisions, and context. Focus on facts and actionable information."
                },
                {
                    "role": "user",
                    "content": f"Summarize this conversation history:\n\n{history_text}"
                }
            ]
            
            # Call OpenAI to generate summary
            completion = self.client.chat.completions.create(
                model=self.model,
                messages=summary_messages,
                temperature=0.3,  # Lower temperature for consistent summaries
                max_tokens=500
            )
            
            new_summary = completion.choices[0].message.content.strip()
            
            # Append to existing summary if any
            if self.summary_memory:
                self.summary_memory = f"{self.summary_memory}\n\n[NEW SESSION]\n{new_summary}"
            else:
                self.summary_memory = new_summary
            
            # Save summary to file
            with open(self.summary_path, 'w') as f:
                json.dump({
                    "summary": self.summary_memory,
                    "last_updated": time.time(),
                    "turns_summarized": len(self.turn_history)
                }, f, indent=2)
            
            # Clear turn history after summarizing
            self.turn_history = []
            
            rospy.loginfo("[LLMAgent] Memory summary updated successfully")
            
        except Exception as e:
            rospy.logerr("[LLMAgent] Failed to update summary: %s", str(e))
        
    def handle_chat_request(self, req):
        """Handle incoming chat completion requests."""
        response = ChatCompletionResponse()
        
        try:
            # Validate request
            if not req.prompt or req.prompt.strip() == "":
                response.success = False
                response.error_message = "Empty prompt provided"
                rospy.logwarn("[LLMAgent] Empty prompt received")
                return response
            
            # Use provided values or defaults
            temperature = req.temperature if req.temperature > 0 else self.default_temperature
            max_tokens = req.max_tokens if req.max_tokens > 0 else self.default_max_tokens
            system_message = req.system_message if req.system_message else self.default_system_message
            
            # Process multimodal inputs
            image_content = None
            text_file_content = None
            model_to_use = self.model
            
            # Handle text file
            if req.text_file_path:
                rospy.loginfo("[LLMAgent] Loading text file: %s", req.text_file_path)
                if not os.path.exists(req.text_file_path):
                    response.success = False
                    response.error_message = f"Text file not found: {req.text_file_path}"
                    rospy.logwarn("[LLMAgent] %s", response.error_message)
                    return response
                text_file_content = self._read_text_file(req.text_file_path)
                rospy.loginfo("[LLMAgent] Text file loaded (%d chars)", len(text_file_content))
            
            # Handle image (path or URL)
            if req.image_path or req.image_url:
                # Switch to vision model if needed
                if req.use_vision_model or "gpt-4o" not in model_to_use:
                    model_to_use = "gpt-4o"
                    rospy.loginfo("[LLMAgent] Switching to vision model: %s", model_to_use)
                
                if req.image_path:
                    rospy.loginfo("[LLMAgent] Loading image: %s", req.image_path)
                    if not os.path.exists(req.image_path):
                        response.success = False
                        response.error_message = f"Image file not found: {req.image_path}"
                        rospy.logwarn("[LLMAgent] %s", response.error_message)
                        return response
                    
                    # Encode image to base64
                    base64_image = self._encode_image(req.image_path)
                    mime_type = self._get_image_mime_type(req.image_path)
                    
                    image_content = {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:{mime_type};base64,{base64_image}"
                        }
                    }
                    rospy.loginfo("[LLMAgent] Image encoded successfully")
                
                elif req.image_url:
                    rospy.loginfo("[LLMAgent] Using image URL: %s", req.image_url)
                    image_content = {
                        "type": "image_url",
                        "image_url": {
                            "url": req.image_url
                        }
                    }
            
            # Log request
            rospy.loginfo("[LLMAgent] Processing chat request:")
            rospy.loginfo("[LLMAgent]   Model: %s", model_to_use)
            rospy.loginfo("[LLMAgent]   Prompt: %s", req.prompt[:100] + "..." if len(req.prompt) > 100 else req.prompt)
            rospy.loginfo("[LLMAgent]   Temperature: %.2f", temperature)
            rospy.loginfo("[LLMAgent]   Max tokens: %d", max_tokens)
            rospy.loginfo("[LLMAgent]   Has image: %s", bool(image_content))
            rospy.loginfo("[LLMAgent]   Has text file: %s", bool(text_file_content))
            if self.enable_history:
                rospy.loginfo("[LLMAgent]   History size: %d turns", len(self.turn_history))
                rospy.loginfo("[LLMAgent]   Has summary: %s", bool(self.summary_memory))
            
            # Prepare messages with history and multimodal content
            messages = self._build_messages(req.prompt, system_message, image_content, text_file_content)
            
            # Determine if we should use structured output
            use_structured = self.use_structured_output or "store" in req.prompt.lower()
            
            # Call OpenAI API
            if use_structured and image_content:
                rospy.loginfo("[LLMAgent] Using structured output (StoreIdentification)")
                completion = self.client.beta.chat.completions.parse(
                    model=model_to_use,
                    messages=messages,
                    temperature=temperature,
                    max_tokens=max_tokens,
                    response_format=StoreIdentification
                )
                
                # Extract structured response
                parsed = completion.choices[0].message.parsed
                ai_response = json.dumps({
                    "store_type": parsed.store_type,
                    "confidence": parsed.confidence,
                    "reasoning": parsed.reasoning
                }, indent=2)
            else:
                completion = self.client.chat.completions.create(
                    model=model_to_use,
                    messages=messages,
                    temperature=temperature,
                    max_tokens=max_tokens
                )
                
                # Extract response
                ai_response = completion.choices[0].message.content
            
            # Save to history
            if self.enable_history:
                self._save_entry("user", req.prompt)
                self._save_entry("assistant", ai_response)
                # Update summary if needed
                self._update_summary()
            
            response.success = True
            response.response = ai_response
            response.error_message = ""
            
            rospy.loginfo("[LLMAgent] Response generated successfully (%d tokens)", 
                         completion.usage.completion_tokens)
            rospy.logdebug("[LLMAgent] Response: %s", ai_response[:100] + "..." if len(ai_response) > 100 else ai_response)
            
        except Exception as e:
            response.success = False
            response.response = ""
            response.error_message = str(e)
            rospy.logerr("[LLMAgent] Error processing chat request: %s", str(e))
        
        return response

    def run(self):
        """Keep the node running."""
        rospy.loginfo("[LLMAgent] Running LLM Agent...")
        rospy.spin()


def main():
    try:
        node = LLMAgentNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[LLMAgent] Node shutting down")
    except Exception as e:
        rospy.logerr("[LLMAgent] Fatal error: %s", str(e))


if __name__ == "__main__":
    main()