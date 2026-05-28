from control_brain import ControlBrain


def print_menu():
    print()
    print("========================================")
    print("          APEX ROVER JACK TEST")
    print("========================================")
    print("Front jack:")
    print("  fe     = front extend")
    print("  fr     = front retract")
    print("  fs     = front stop")
    print()
    print("Rear jack:")
    print("  re     = rear extend")
    print("  rr     = rear retract")
    print("  rs     = rear stop")
    print()
    print("Both:")
    print("  jstop  = stop both jacks")
    print("  q      = quit")
    print()
    print("TEST SAFELY: write extend, wait about 1 second, then write stop.")
    print("========================================")
    print()


def main():
    brain = ControlBrain()
    brain.connect_all()
    print_menu()

    while True:
        cmd = input("JackTest> ").strip().lower()

        if cmd == "fe":
            brain.front_jack_extend()
        elif cmd == "fr":
            brain.front_jack_retract()
        elif cmd == "fs":
            brain.front_jack_stop()
        elif cmd == "re":
            brain.rear_jack_extend()
        elif cmd == "rr":
            brain.rear_jack_retract()
        elif cmd == "rs":
            brain.rear_jack_stop()
        elif cmd == "jstop":
            brain.stop_all_jacks()
        elif cmd == "q":
            brain.stop_all_jacks()
            brain.shutdown()
            break
        elif cmd == "":
            continue
        else:
            print("[ERROR] Unknown command")
            print_menu()


if __name__ == "__main__":
    main()
