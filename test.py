from thorpy.comm.discovery import discover_stages

if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    s = stages[0]
    
    # s.home()
    
    # s.print_state()
    
    import IPython
    IPython.embed()
    
    #del s, stages

    # s._set_homeparams(100, s.home_direction, s.home_limit_switch, s.home_offset_distance)
    # s.home()
    # s.print_state()
    # s._set_velparams(0, 25000, s.acceleration)

    # p = s._port
    # p.send_message(MGMSG_MOT_MOVE_ABSOLUTE_long(s._chan_ident, 10000000))


