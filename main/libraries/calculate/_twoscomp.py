def twoscomp_8(value: int) -> int:
    return ((value ^ 0xff) + 1) & 0xff