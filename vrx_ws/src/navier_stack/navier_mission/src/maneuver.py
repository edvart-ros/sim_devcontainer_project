from __future__ import annotations
from situation import Position


def driveBetween(left:Position,right:Position,margin:float = 1.0)->list[Position]:
    midpoint:Position = (left+right)/2
    diff:Position = _rotate90degreees(left-midpoint)
    diffScaled = diff/abs(diff)*margin
    return [midpoint-diffScaled,midpoint+diffScaled] #CHECK SIGNS

def _rotate90degreees(vector:Position, clockwise:bool = True)->Position:
    if clockwise:
        return Position(vector.getY(),-vector.getX())
    return Position(-vector.getY(),vector.getX())

def goAround(current:Position,middle:Position,margin:float = 2.0,clockwise:bool = True)->list[Position]:
    diff:Position = current-middle
    diffScaled:Position = diff/abs(diff)*margin
    front1:Position = middle+diffScaled
    front2:Position = front1 + Position(0,0)
    side1:Position = middle+_rotate90degreees(diffScaled,clockwise = clockwise)
    side2:Position = middle+_rotate90degreees(diffScaled,clockwise = not clockwise)
    back:Position = middle-diffScaled
    return [front1,side1,back,side2,front2]