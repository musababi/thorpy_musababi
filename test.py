#!/usr/bin/env python3
from thorpy.comm.discovery import discover_stages

if __name__ == '__main__':
    from thorpy.message import *
    
    stages = list(discover_stages())
    print(stages)
    s0 = stages[0]
    s1 = stages[1]

    s0._set_homeparams(10000, s0.home_direction, s0.home_limit_switch, s0.home_offset_distance)
    s1._set_homeparams(10000, s1.home_direction, s1.home_limit_switch, s1.home_offset_distance)

    s0.home_non_blocking()
    s1.home_non_blocking()

    p0 = s0._port
    p1 = s1._port
    
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


