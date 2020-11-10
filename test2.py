#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages

def discover_s():
    # import usb
    import os
    from thorpy.comm.port import Port
    from serial.tools.list_ports import comports
    import platform
    
    serial_ports = [(x[0], x[1], dict(y.split('=', 1) for y in x[2].split(' ') if '=' in y)) for x in comports()]
    print(serial_ports)
    
    for ser in serial_ports:
        print('a serial port selected. \n')
        print(ser[0])
        print('\n')
        if ser[1] == 'APT Stepper Motor Controller':
            p = Port.create(ser[0], ser[2].get('SER', None))
            print('connected to one stage\n')
            for stage in p.get_stages().values():
                yield stage

if __name__ == '__main__':
    
    stages = list(discover_s())
    # print(stages)
    # s = stages[0]
    
    # s.home()
    
    # s.print_state()
    
    import IPython
    IPython.embed()
    
    #del s, stages
