from pydantic import BaseModel
from typing import Literal

class StoreIdentification(BaseModel):
    """Structured output for store type identification."""
    store_type: Literal["cafe", "pharmacy", "convenience store", "restaurant", "unknown"]
    confidence: Literal["high", "medium", "low"]
    reasoning: str