import sys
import m5
import argparse
from m5.objects import *


from system import *

m5.util.addToPath('/home/murattokez/riscv-fs-gem5/gem5/configs')
from common import SimpleOpts



def parse_options():
    parser = argparse.ArgumentParser(description='Runs Linux boot test with'
                'RISCV. Expects the disk image to call the simulator exit'
                'event after boot.')
    parser.add_argument("--kernel", help='Path to the bootloader(i used bbl)'
                                        'binary with kernel payload')
    parser.add_argument("--disk", help="Path to the disk image to boot")
    parser.add_argument("--num_cpus", type=int, help="Number of CPU cores")

    return parser.parse_args()


if __name__ == "__m5_main__":
    args = parse_options()

    system = RiscvSystem(args.kernel, args.disk, args.num_cpus)

    # Read in the script file passed in via an option.
    # This file gets read and executed by the simulated system after boot.
    # Note: The disk image needs to be configured to do this.

    # set up the root SimObject and start the simulation
    root = Root(full_system = True, system = system)

    # instantiate all of the objects we've created above
    m5.instantiate()

    # Keep running until we are done.
    print( "Running the simulation")
    exit_event = m5.simulate()
    print( 'Exiting @ tick %i because %s' % (m5.curTick(),
                                            exit_event.getCause()))