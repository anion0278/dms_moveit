#!/usr/bin/env python

import curses, time

def key_hook(stdscr):
    stdscr.nodelay(True)
    return stdscr.getch()

def choose_option_by_key(keys_with_description_dict, use_delay = False):
    print("Choose action by pressing button:")
    for k,v in keys_with_description_dict.items():
        print("* " + v + " -> [" + k + "]")
    print("< Press ESC to quit the program >")

    curses.initscr() # discard anything that was not caught in the meantime (important!)
    curses.flushinp()

    while True:
        pressed_key_code = curses.wrapper(key_hook)
        if pressed_key_code == 27: # esc
            print("ESC - Keyboard interrupt!")
            raise KeyboardInterrupt 
        # ic(pressed_key_code)
        pressed_key = None
        try:
            pressed_key = chr(pressed_key_code)
        except:
            pass
        if pressed_key != None:
            if  pressed_key in keys_with_description_dict.keys():
                print("Chosen option: %s" % keys_with_description_dict[pressed_key])
                break
            else: print("Unrecognized command")
        time.sleep(0.1)
    if use_delay:
        time.sleep(1) # to make sure that user does not hold the button
    return pressed_key