#!/usr/bin/env python3
import rospy
import os
import json
import time
from openai import OpenAI
from pilot.srv import ChatCompletion, ChatCompletionResponse

# History management constants
MAX_HISTORY = 20        # Maximum conversation turns to keep in memory
SUMMARY_TRIGGER = 10    # Trigger summary after this many turns


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
    
    def _build_messages(self, user_prompt, system_message):
        """Build message list with history and summary."""
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
        
        # Add current user prompt
        messages.append({"role": "user", "content": user_prompt})
        
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
            
            # Log request
            rospy.loginfo("[LLMAgent] Processing chat request:")
            rospy.loginfo("[LLMAgent]   Prompt: %s", req.prompt[:100] + "..." if len(req.prompt) > 100 else req.prompt)
            rospy.loginfo("[LLMAgent]   Temperature: %.2f", temperature)
            rospy.loginfo("[LLMAgent]   Max tokens: %d", max_tokens)
            if self.enable_history:
                rospy.loginfo("[LLMAgent]   History size: %d turns", len(self.turn_history))
                rospy.loginfo("[LLMAgent]   Has summary: %s", bool(self.summary_memory))
            
            # Prepare messages with history
            messages = self._build_messages(req.prompt, system_message)
            
            # Call OpenAI API
            completion = self.client.chat.completions.create(
                model=self.model,
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