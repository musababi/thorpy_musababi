from thorpy.comm.discovery import discover_stages

if __name__ == '__main__':
    import usb
    import os
    from thorpy.comm.port import Port
    from serial.tools.list_ports import comports
    import platform

    serial_ports = [(x[0], x[1], dict(y.split('=', 1) for y in x[2].split(' ') if '=' in y)) for x in comports()]
    print('serial ports: ', serial_ports, '\n\n')
    
    # for dev in usb.core.find(find_all=True, custom_match= lambda x: x.bDeviceClass != 9):
    #     try:
    #         #FIXME: this avoids an error related to https://github.com/walac/pyusb/issues/139
    #         #FIXME: this could maybe be solved in a better way?
    #         dev._langids = (1033, )
    #         # KDC101 3-port is recognized as FTDI in newer kernels
    #         if not (dev.manufacturer == 'Thorlabs' or dev.manufacturer == 'FTDI'):
    #             continue
    #     except usb.core.USBError:
    #         print('get the exception')
    #         print(dev)
    #         continue
        
    #     if platform.system() == 'Linux':
    #         print('\n\nthe s/n: ', dev.serial_number)
    #         port_candidates = [x[0] for x in serial_ports if x[2].get('SER', None) == dev.serial_number]
    #     else:
    #         raise NotImplementedError("Implement for platform.system()=={0}".format(platform.system()))
        
    #     assert len(port_candidates) == 1
        
    #     port = port_candidates[0]
    #     print('\n\nthe port: ', port)

    #     p = Port.create(port, dev.serial_number)
    #     print('\n\nports: ', p)
    #     print('\n\nget stage values -->', p.get_stages().values())

        # for stage in p.get_stages().values():
        #     yield stage
    
    # stages = list(discover_stages())
    # print(stages)
    # s = stages[0]
    
    # s.home()
    
    # s.print_state()
    
    import IPython
    IPython.embed()
    
    #del s, stages
