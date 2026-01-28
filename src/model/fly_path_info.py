from pydantic import BaseModel
from typing import List, Optional

class FlyPathPoint(BaseModel):
    lon: float
    lat: float
    height: float
    time: str

class FlyPathInfo(BaseModel):
    id: str
    path: List[FlyPathPoint]
    name: Optional[str] = None
    color: Optional[str] = None
    isMaster: bool = False
