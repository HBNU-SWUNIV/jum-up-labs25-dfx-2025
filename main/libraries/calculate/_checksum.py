from typing import Iterable

def checksum_8(data: Iterable[int]) -> int:
    return sum(data, 0) & 0xFF