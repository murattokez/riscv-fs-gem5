import argparse
import sys
from os import path

import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.util import addToPath, fatal, warn
from m5.util.fdthelper import *


from Caches import *



def generateMemNode(state, mem_range):
    node = FdtNode("memory@%x" % int(mem_range.start))
    node.append(FdtPropertyStrings("device_type", ["memory"]))
    node.append(
        FdtPropertyWords(
            "reg",
            state.addrCells(mem_range.start)
            + state.sizeCells(mem_range.size()),
        )
    )
    return node


def generateDtb(system):
    state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
    root = FdtNode("/")
    root.append(state.addrCellsProperty())
    root.append(state.sizeCellsProperty())
    root.appendCompatible(["riscv-virtio"])
    root.append(FdtPropertyStrings("model", ["riscv-virtio,gem5"]))

    for mem_range in system.mem_ranges:
        root.append(generateMemNode(state, mem_range))

    sections = [*system.cpu, system.platform]

    for section in sections:
        for node in section.generateDeviceTree(state):
            if node.get_name() == root.get_name():
                root.merge(node)
            else:
                root.append(node)
    

    fdt = Fdt()
    fdt.add_rootnode(root)
    fdt.writeDtsFile(path.join(m5.options.outdir, "device.dts"))
    fdt.writeDtbFile(path.join(m5.options.outdir, "device.dtb"))



class RiscvSystem(System):

    def __init__(self, kernel, disk, num_cpus):
        super(RiscvSystem, self).__init__()

        # Set up the clock domain and the voltage domain
        self.clk_domain = SrcClockDomain()
        self.clk_domain.clock = '3GHz'
        self.clk_domain.voltage_domain = VoltageDomain()
        self.cpu_clk_domain = SrcClockDomain()
        self.cpu_clk_domain.voltage_domain = VoltageDomain()
        self.cpu_clk_domain.clock = '3GHz'

        # DDR memory range starts from base address 0x80000000
        # based on [1]
        self.mem_ranges = [AddrRange(start=0x80000000, size='1024MB')]

        # Create the main memory bus
        # This connects to main memory
        self.membus = SystemXBar(width = 64) 

        self.membus.badaddr_responder = BadAddr()
        self.membus.default = Self.badaddr_responder.pio

        # Set up the system port for functional access from the simulator
        self.system_port = self.membus.cpu_side_ports

        # Create the CPUs for our system.
        self.createCPU(num_cpus)

        # HiFive platform
        # This is based on a HiFive RISCV board and has
        # only a limited number of devices so far i.e.
        # PLIC, CLINT, UART, VirtIOMMIO
        self.platform = HiFive()

        # create and intialize devices currently supported for RISCV
        self.initDevices(self.membus, disk)

        # Create the cache heirarchy for the system.
        self.createCacheHierarchy(num_cpus)

        # Create the memory controller
        self.createMemoryControllers()

        self.setupInterrupts(num_cpus)

        # using RiscvLinux as the base full system workload
        self.workload = RiscvLinux()

        #bbl dosyası yani bootloader ve linux kernel'i buraya gelecek
        self.workload.object_file = kernel

        # Generate DTB (from configs/example/riscv/fs_linux.py)
        generateDtb(self)
        self.workload.dtb_filename = path.join(m5.options.outdir, 'device.dtb')
        # Default DTB address if bbl is bulit with --with-dts option
        self.workload.dtb_addr = 0x87E00000

        # Linux boot command flags
        kernel_cmd = [
            "console=ttyS0",
            "root=/dev/vda",
            "ro"
        ]
        self.workload.command_line = " ".join(kernel_cmd)


    def createCPU(self, num_cpus):

        #cpu_id=i yapmazsan fatal: Value -1 doesn't fit in 1 cells hatasi veriyor

        self.cpu = [TimingSimpleCPU(cpu_id = i) for i in range(num_cpus)] # num_cpu adet O3 cpu

        self.mem_mode = 'timing' # Use timing accesses
         
        for cpu in self.cpu:
            cpu.createThreads()
        


    def createCacheHierarchy(self, num_cpus):

        # We can't directly connect the L1 caches to the L2 cache since the L2 cache only expects a single port to connect to it.
        # Therefore, we need to create an L2 bus toconnect out L1 caches to the L2 cache. Then we can use our helper function to connect the L1 caches to the L2 bus.
        self.l2bus = L2XBar()
        self.l3bus = L2XBar() #herm de tol2 bus olarak kullanılacak main memorye bu baglandıgı icin

        for i in range(num_cpus):
            self.cpu[i].icache = L1ICache()
            self.cpu[i].dcache = L1DCache()

            self.cpu[i].mmucache = L1ICache() ##mmu cachek

            #icache ve dcache l2 busa baglanir
            self.cpu[i].icache.mem_side = self.l2bus.cpu_side_ports
            self.cpu[i].dcache.mem_side = self.l2bus.cpu_side_ports

            #l1'ler cpu'ya baglanir
            self.cpu[i].icache.cpu_side = self.cpu[i].icache_port
            self.cpu[i].dcache.cpu_side = self.cpu[i].dcache_port

            #mmucache icin mmubus olusturulur
            self.cpu[i].mmucache.mmubus = L2XBar()

            #mmucache mmubus'a baglanir
            self.cpu[i].mmucache.cpu_side = self.cpu[i].mmucache.mmubus.mem_side_ports
            
            # l1'lere baglı mmucache l2bus'a baglanir
            self.cpu[i].mmucache.mem_side = self.l2bus.cpu_side_ports

            self.cpu[i].mmu.connectWalkerPorts(self.cpu[i].mmucache.mmubus.cpu_side_ports, self.cpu[i].mmucache.mmubus.cpu_side_ports)
        
       


        
        


        # create our two l2 cache and connect it to the L2 bus
        self.l2cache1 = L2Cache()

        self.l2cache1.connectCPUSideBus(self.l2bus)
        
        # create out l3 cache and connect it to the L3 bus and the memory bus
        self.l3cache = L3Cache()
        self.l3cache.connectCPUSideBus(self.l3bus)

        # Connect the L2 cache to l3bus
        self.l2cache1.connectMemSideBus(self.l3bus)

        #l3'u main memorye bagla
        self.l3cache.connectMemSideBus(self.membus)

        
        


            

    def createMemoryControllers(self):
        self.mem_cntrls = [
            MemCtrl(dram = DDR3_1600_8x8(range = self.mem_ranges[0]),
                    port = self.membus.mem_side_ports
                     )
        ]
        


    def setupInterrupts(self,num_cpus):
        for cpu in self.cpu:
            cpu.createInterruptController()


    def initDevices(self, membus, disk):

            self.iobus = IOXBar()

            

            self.platform.plic.n_contexts = 4 * 2

            # Attach the PLIC (platform level interrupt controller)
            # to the platform. This initializes the PLIC with
            # interrupt sources coming from off chip devices
            self.platform.attachPlic()


            self.platform.clint.num_threads = 4


            # Set the frequency of RTC (real time clock) used by
            # CLINT (core level interrupt controller).
            # This frequency is 1MHz in SiFive's U54MC.
            # Setting it to 100MHz for faster simulation (from riscv/fs_linux.py)
            self.platform.rtc = RiscvRTC(frequency=Frequency("100MHz"))


            # RTC sends the clock signal to CLINT via an interrupt pin.
            self.platform.clint.int_pin = self.platform.rtc.int_pin
            


            # VirtIOMMIO
            image = CowDiskImage(child=RawDiskImage(
                read_only=True), read_only=False)
            
            image.child.image_file = disk
            
            # using reserved memory space
            self.platform.disk = RiscvMmioVirtIO(
                vio=VirtIOBlock(image=image),
                interrupt_id=0x8,
                pio_size=4096,
                pio_addr=0x10008000,
            )

            self._on_chip_devices = [self.platform.clint, self.platform.plic]
            # From riscv/fs_linux.py
            uncacheable_range = [
                *self.platform._on_chip_ranges(),
                *self.platform._off_chip_ranges()
            ]
            # PMA (physical memory attribute) checker is a hardware structure
            # that ensures that physical addresses follow the memory permissions

            # PMA checker can be defined at system-level (system.pma_checker)
            # or MMU-level (system.cpu[0].mmu.pma_checker). It will be resolved
            # by RiscvTLB's Parent.any proxy

            for cpu in self.cpu:
                cpu.mmu.pma_checker = PMAChecker(uncacheable=uncacheable_range)

            self.bridge = Bridge(delay='50ns')
            self.bridge.mem_side_port = self.iobus.cpu_side_ports
            self.bridge.cpu_side_port = self.membus.mem_side_ports
            self.bridge.ranges = self.platform._off_chip_ranges()

            # Connecting on chip and off chip IO to the mem
            # and IO bus
            self.platform.attachOnChipIO(self.membus)
            self.platform.attachOffChipIO(self.iobus)

            
            
            self.platform.pci_host.pio = self.iobus.mem_side_ports # pci port hatasi içinnn



    
