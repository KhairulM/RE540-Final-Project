# LLM Agent Node

OpenAI-powered chat node for ROS with service interface.

## Setup

1. **Install OpenAI Python package:**
```bash
pip install openai
```

2. **Set your OpenAI API key:**
```bash
export OPENAI_API_KEY='your-api-key-here'
```

To make it permanent, add to `~/.bashrc`:
```bash
echo "export OPENAI_API_KEY='your-api-key-here'" >> ~/.bashrc
source ~/.bashrc
```

## Running the Node

```bash
# Source workspace
source /home/khairulm/re540/final_project_ws/devel/setup.bash

# Run the node
rosrun pilot llm_agent_node.py
```

### With custom parameters:
```bash
rosrun pilot llm_agent_node.py _model:=gpt-4 _default_temperature:=0.9 _default_max_tokens:=1000
```

## Using the Service

### Command Line

**Basic usage:**
```bash
rosservice call /llm_chat "prompt: 'What is the capital of France?'
system_message: ''
temperature: 0.0
max_tokens: 0"
```

**With custom parameters:**
```bash
rosservice call /llm_chat "prompt: 'Explain quantum computing in simple terms'
system_message: 'You are a science educator for children'
temperature: 0.8
max_tokens: 200"
```

### Python Code

```python
#!/usr/bin/env python3
import rospy
from pilot.srv import ChatCompletion

rospy.init_node('chat_client')

# Wait for service
rospy.wait_for_service('/llm_chat')

# Create service proxy
chat = rospy.ServiceProxy('/llm_chat', ChatCompletion)

try:
    # Simple call with defaults
    response = chat(
        prompt="What is machine learning?",
        system_message="",  # Use default
        temperature=0.0,    # Use default
        max_tokens=0        # Use default
    )
    
    if response.success:
        print("Response:", response.response)
    else:
        print("Error:", response.error_message)
        
    # Custom call
    response = chat(
        prompt="Explain SLAM for robots",
        system_message="You are a robotics expert",
        temperature=0.7,
        max_tokens=300
    )
    
    if response.success:
        print("Response:", response.response)
        
except rospy.ServiceException as e:
    print("Service call failed:", e)
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~model` | string | `gpt-4o-mini` | OpenAI model to use |
| `~default_temperature` | float | `0.7` | Default temperature (0-2) |
| `~default_max_tokens` | int | `500` | Default max response tokens |
| `~default_system_message` | string | `"You are a helpful AI assistant for a mobile robot with long-term memory."` | Default system prompt |
| `~enable_history` | bool | `true` | Enable conversation history and memory |
| `~max_history` | int | `20` | Maximum conversation turns to keep |
| `~summary_trigger` | int | `10` | Trigger summary after this many turns |

## Service Interface

**Request:**
- `prompt` (string): The user's message/question
- `system_message` (string): System context (optional, uses default if empty)
- `temperature` (float32): Sampling temperature (optional, uses default if 0)
- `max_tokens` (int32): Max response length (optional, uses default if 0)

**Response:**
- `success` (bool): Whether the request succeeded
- `response` (string): The AI's response (empty on failure)
- `error_message` (string): Error details (empty on success)

## Examples

### Navigation Assistant
```bash
rosservice call /llm_chat "prompt: 'The robot is at (0, 0) and needs to reach (5, 3). There is an obstacle at (2, 1). What path should it take?'
system_message: 'You are a navigation planner for mobile robots. Provide concise, actionable navigation instructions.'
temperature: 0.5
max_tokens: 200"
```

### Task Planning
```bash
rosservice call /llm_chat "prompt: 'Plan the steps to pick up a cup from a table and place it in a cupboard'
system_message: 'You are a task planning system for a mobile manipulator robot. Break tasks into simple sequential steps.'
temperature: 0.3
max_tokens: 300"
```

## Conversation History & Memory

The node automatically maintains conversation history with intelligent summarization:

### How It Works

1. **Recent History**: Keeps the last 20 conversation turns (configurable)
2. **Auto-Summarization**: After 10 turns, automatically creates a summary
3. **Long-term Memory**: Summaries are preserved across sessions
4. **Context Awareness**: Each request includes relevant history and summaries

### History Files

History is stored in `pilot/scripts/history/`:
- `session_<timestamp>.jsonl` - Current session conversation log
- `summary.json` - Cumulative memory summary

### Example With History

```bash
# First interaction
rosservice call /llm_chat "prompt: 'My name is Alex and I work on robot navigation'
system_message: ''
temperature: 0.0
max_tokens: 0"

# Later interaction - remembers context
rosservice call /llm_chat "prompt: 'What did I say my name was?'
system_message: ''
temperature: 0.0
max_tokens: 0"
# Response: "You said your name is Alex."
```

### Disabling History

To run without memory (stateless mode):
```bash
rosrun pilot llm_agent_node.py _enable_history:=false
```

### Clearing History

To start fresh:
```bash
rm -rf /path/to/pilot/scripts/history/
```

## Troubleshooting

### "OPENAI_API_KEY not found"
Make sure you've exported the environment variable before running the node:
```bash
export OPENAI_API_KEY='sk-...'
```

### "Failed to initialize OpenAI client"
Check that:
1. Your API key is valid
2. You have internet connectivity
3. The `openai` package is installed: `pip install openai`

### Service call timeout
The OpenAI API might be slow for large requests. You can increase the timeout:
```python
chat = rospy.ServiceProxy('/llm_chat', ChatCompletion, persistent=True)
chat.wait_for_service(timeout=60)
```
