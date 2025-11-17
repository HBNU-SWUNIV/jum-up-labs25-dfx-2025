### Imports ###
from typing import Tuple
from struct import Struct
from serial import Serial
from ..calculate import twoscomp_8, checksum_8


### Class ###
class OpenCRSerial:
    # exceptions
    class ShortResponseError(Exception):
        '''response 길이가 적절치 않은 에러'''
    class ResponseHeaderError(Exception):
        '''response 헤더가 request 헤더와 같지 않은 에러'''
    class CheckSumError(Exception):
        '''체크섬 실패 에러'''
    class RetcodeError(Exception):
        '''retcode가 성공(0x00)이 아님'''
    
    
    
    # formats
    DEVADDR_SIZE    = 1
    FUNCCODE_SIZE   = 1
    RETCODE_SIZE    = 1
    CHECKSUM_SIZE   = 1
    TX_HEADER_SIZE  = DEVADDR_SIZE + FUNCCODE_SIZE
    RX_HEADER_SIZE  = TX_HEADER_SIZE
    RX_TAIL_SIZE    = RETCODE_SIZE + CHECKSUM_SIZE
    
    ENDIAN          = '<'
    TX_HEADER       = f'{TX_HEADER_SIZE}B'
    RX_HEADER       = f'{RX_HEADER_SIZE}B'
    RX_TAIL         = f'{RX_TAIL_SIZE}B'
    
    Param_MicroPhoto_1                = ''
    Param_MicroPhoto_2                = ''
    Param_TDTL                        = ''
    Param_dxl_torqueOn                = '1B'
    Param_dxl_torqueOff               = '1B'
    Param_dxl_goalPosition            = '1B1l'
    Param_dxl_getPresentPositionData  = '1B'
    Param_dxl_init                    = '1B1l1l'
    
    Return_MicroPhoto_1               = '1B'
    Return_MicroPhoto_2               = '1B'
    Return_TDTL                       = '1B'
    Return_dxl_torqueOn               = ''
    Return_dxl_torqueOff              = ''
    Return_dxl_goalPosition           = ''
    Return_dxl_getPresentPositionData = '1l'
    Return_dxl_init                   = ''
    
    MSGFMT: Tuple[Tuple[Struct, Struct]] = (
        # (packfmt(for transmit)(tail 빼고), unpackfmt(for receive))
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_MicroPhoto_1}')              , Struct(f'{ENDIAN}{RX_HEADER}{Return_MicroPhoto_1}{RX_TAIL}')              ),
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_MicroPhoto_2}')              , Struct(f'{ENDIAN}{RX_HEADER}{Return_MicroPhoto_2}{RX_TAIL}')              ),
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_TDTL}')                      , Struct(f'{ENDIAN}{RX_HEADER}{Return_TDTL}{RX_TAIL}')                      ),
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_dxl_torqueOn}')              , Struct(f'{ENDIAN}{RX_HEADER}{Return_dxl_torqueOn}{RX_TAIL}')              ),
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_dxl_torqueOff}')             , Struct(f'{ENDIAN}{RX_HEADER}{Return_dxl_torqueOff}{RX_TAIL}')             ),
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_dxl_goalPosition}')          , Struct(f'{ENDIAN}{RX_HEADER}{Return_dxl_goalPosition}{RX_TAIL}')          ),
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_dxl_getPresentPositionData}'), Struct(f'{ENDIAN}{RX_HEADER}{Return_dxl_getPresentPositionData}{RX_TAIL}')),
        (Struct(f'{ENDIAN}{TX_HEADER}{Param_dxl_init}')                  , Struct(f'{ENDIAN}{RX_HEADER}{Return_dxl_init}{RX_TAIL}')                  ),
    )
    
    # constants
    # VALUE_DEVADDR = 0xF0
    VALUE_FUNCCODE_microphoto_1               = 0x00
    VALUE_FUNCCODE_microphoto_2               = 0x01
    VALUE_FUNCCODE_tdtl                       = 0x02
    VALUE_FUNCCODE_dxl_torqueOn               = 0x03
    VALUE_FUNCCODE_dxl_torqueOff              = 0x04
    VALUE_FUNCCODE_dxl_goalPosition           = 0x05
    VALUE_FUNCCODE_dxl_getPresentPositionData = 0x06
    VALUE_FUNCCODE_dxl_init                   = 0x07
    
    VALUE_RETCODE_CALLBACK_SUCCESS = 0x00
    VALUE_RETCODE_CALLBACK_FAIL    = 0x01
    
    VALUE_MAIN_PWM_FACTOR  = 2**17
    VALUE_SERVO_PWM_FACTOR = 2**17
    
    VALUE_ACCADC_RESOLUTION = 8.0/32768.0
    
    
    def __init__(self, port: str, baudrate: int, timeout: float, devaddr: int):
        self.__serial           = Serial(port, baudrate, 8, 'N', 1, timeout=timeout, write_timeout=timeout)
        self.__devaddr          = devaddr
    
    
    
    def __communicate(self, devaddr, funccode, *data) -> tuple:
        txfmt: Struct = self.MSGFMT[funccode][0]
        rxfmt: Struct = self.MSGFMT[funccode][1]
        
        payload = [devaddr, funccode, *data]
        payload = txfmt.pack(*payload)
        payload += twoscomp_8(checksum_8(payload)).to_bytes(1, 'little')
        
        self.__serial.reset_input_buffer()
        self.__serial.reset_output_buffer()
        self.__serial.write(payload)
        # print(f'write: 0x {payload.hex(" ")}')
        
        response = self.__serial.read(rxfmt.size)
        # print(f'read : 0x {response.hex(" ")}')
        
        if len(response) != rxfmt.size:
            raise self.ShortResponseError(f'response: 0x {response.hex(" ")}')
        if checksum_8(response) != 0:
            raise self.CheckSumError(f'response: 0x {response.hex(" ")}')
        if payload[:self.TX_HEADER_SIZE] != response[:self.RX_HEADER_SIZE]:
            raise self.ResponseHeaderError(f'response: 0x {response.hex(" ")}')
        if int.from_bytes(response[-self.RX_TAIL_SIZE: -self.CHECKSUM_SIZE], 'little') != self.VALUE_RETCODE_CALLBACK_SUCCESS:
            raise self.RetcodeError(f'response: 0x {response.hex(" ")}')
        
        return rxfmt.unpack(response)[self.RX_HEADER_SIZE: -self.RX_TAIL_SIZE]

    def microphoto_1(self) -> None:
        return self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_microphoto_1)
    
    def microphoto_2(self) -> None:
        return self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_microphoto_2)

    def tdtl(self) -> None:
        return self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_tdtl)

    def dxl_torqueOn(self, id: int) -> None:
        return self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_dxl_torqueOn, id)
    
    def dxl_torqueOff(self, id: int) -> None:
        return self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_dxl_torqueOff, id)
        
    def dxl_goalPosition(self, id: int, value: int) -> None:
        return self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_dxl_goalPosition, id, value)
        
    def dxl_getPresentPositionData(self, id: int) -> int:
        ret = self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_dxl_getPresentPositionData, id)
        return ret[0]
    
    def dxl_init(self, id: int, velocity: int, acceleration: int):
        '''
        해당 메서드로 초기화에 성공하지 않은 id의 다이나믹셀을 조작할 경우 실패할 수 있습니다.
        '''
        return self.__communicate(self.__devaddr, self.VALUE_FUNCCODE_dxl_init, id, velocity, acceleration)