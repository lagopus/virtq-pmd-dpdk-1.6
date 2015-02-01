##virtq-pmd-dpdk and virtio-net-ipc enabled QEMU##

###What is virtq-pmd-dpdk ?###
virtq-pmd-dpdk is [Intel DPDK library](http://dpdk.org) with an additional virtio-based poll-mode driver (we call "virtq PMD") 
which can directly communicate virtual machine "[QEMU](http://wiki.qemu.org/Main_Page)" without bridge-interface, 
such as linux-bridge or tap driver.
For direct communication between a DPDK application and virtual machine, It needs customized qemu emulator with an additional virtual network device (we call "virtio-net-ipc device").
This is based on DPDK version 1.6.0-18.

###What is virtio-net-ipc enabled QEMU ?###
virtio-net-ipc enabled QEMU is qemu emulator with additional virtual network device "virtio-net-ipc device". This qemu emulator directly transfer data between guest and DPDK application with virtq-pmd-dpdk. This is based on QEMU version 1.0.

###What are virtq PMD and virtio-net-ipc device?###

virtq PMD is a poll mode driver of DPDK, and virtio-net-ipc device is a kind of virtio-net device of QEMU. Those are used for establishing a fast path between a DPDK application on the host and a virtio-net driver on the guest. Both virtio-net drivers of linux kernel and virtio-net PMD of DPDK can be the virtio-net driver that virtq PMD connects. Frankly, it is almost similar to vhost-user of QEMU-2.1, but has a few features vhost-user doesn't have.

###Where can I download those from?###

- DPDK-1.6.0-18 with virtq PMD
 - https://github.com/lagopus/virtq-pmd-dpdk
- QEMU-1.0 with virtio-net-ipc
 - https://github.com/lagopus/virtio-net-ipc-qemu1.0

###What are features?###

- One copy data transfer between a DPDK application on the host and the virtio-net driver on the guest.
- The DPDK application on the host or the guest can stop and start anytime, even if while the counter part application still be transferring data.
- QEMU also can stop and start anytime, even if while the DPDK application on the host still be transferring data.

  (QEMU and the DPDK applications on the guest or the host must be stopped gently. See later sections)

###How does it work?###

QEMU can mmap file on hugetlbfs for guest physical memory. virtq PMD and virtio-net-ipc communication works with such a guest physical memory.
The buffers and queues used for data transfer are allocated by a virtio-net driver, and those are allocated in guest physical memory.
virtio-net-ipc device receives those information from a virtio-net driver, then pass those to virq PMD by unix domain socket.
As a result, virtq PMD can access to the buffers and queues directly. This is the basic concept of the communication.

###How to use ?###

- Download
 - QEMU

            $ git clone https://github.com/lagopus/virtio-net-ipc-qemu1.0.git

 - DPDK

            $ git clone https://github.com/lagopus/virtq-pmd-dpdk.git

- Compile
 - QEMU

            $ cd virtio-net-ipc-qemu1.0
            $ ./configure --target-list=x86_64-softmmu
            $ make
            $ sudo make install
            $ ls /usr/local/bin/qemu*
            /usr/local/bin/qemu-ga
            /usr/local/bin/qemu-img
            /usr/local/bin/qemu-io
            /usr/local/bin/qemu-nbd
            /usr/local/bin/qemu-system-x86_64

 - DPDK

            $ cd virtq-pmd-dpdk
            $ make install T=x86_64-default-linuxapp-gcc

- Start QEMU

 Before starting, users needs to mount hugetlbfs. For example, to reserve 4G of hugepage memory in the form of four 1G pages,
 the following options should be passed to the kernel:

        default_hugepagesz=1G hugepagesz=1G hugepages=4

 Once the hugepage memory is reserved, to make the memory available for Intel DPDK use, perform the following steps:

        $ mkdir /mnt/huge
        $ mount -t hugetlbfs nodev /mnt/huge

 Here is an one of example to start guest with virtio-net-ipc-device.

        $ sudo /usr/local/qemu/bin/qemu-system-x86_64 \
          -machine pc-1.0 -cpu host,-x2apic \
          -smp 4,sockets=2,cores=1,threads=1 \
          -m 4096 -mem-path /mnt/huge/libvirt/qemu/ -mem-prealloc \
          -drive file=vm.img,if=none,id=drive-virtio-disk0,format=raw \
          -device virtio-blk-pci,bus=pci.0,addr=0x8,drive=drive-virtio-disk0,id=virtio-disk0,bootindex=2 \
          -device virtio-net-ipc-pci,id=net0,mac=52:54:00:c6:a7:41,bus=pci.0,addr=0x09,nid=0,socketpath=/tmp/virtq,cinterval=1 \
          -net none -enable-kvm -vnc localhost:0

 Users can specify options used by virtio-net-ipc devices. Additionally following options can be specified.
 - `virtio-net-ipc-pci` : Specify the virtual network device is "virtio-net-ipc device" with --device option.
 - `id` : This is a uniq id in qemu internal.
 - `mac` : Specify mac address of each virtual device.
 - `socketpath` : Specify where unix domain socket is. The socket is used by QEMU and virtq PMD.
 - `nid` : Specify a device identifier of the device. nid should be unique among devices connected using same socketpath. It should be started from 0. This id correspond to the suffix number "X" of "eth_virtqX" provided by virtq-pmd-dpdk.
 - `cinterval` : Specify reconnection interval when the counter part application is gone. Every cinterval seconds, virtio-net-ipc device tries to reconnect.


- Start a DPDK application on the guest

 Users can use virtio-net-ipc devices, as if those are virtio-net device on the guest. A virtio-net drver of linux kernel and virtio-net PMD can access those.
 
 If testpmd is used as the guest DPDK application, please type 'port start all' after starting testpmd. Usually all ports are started during initialization of testpmd, but testpmd only waits 1 second to finish. virtio-net-ipc devices are using unix domain socket, so it takes much more time to start port than virtio-net device. As a result, it may not finish in 1 second. In the case, type 'port start all'. This is the command to start port manually.


- Start a DPDK application on the host

 Here is an example of starting DPDK application using virtq PMD. When the application starts, users can use virtq PMDs with "--use-device" option.
 Please see [DPDK manuals](http://dpdk.org/doc/guides/prog_guide/) for detail of this option.

        $ sudo RTE_PMD_VIRTQ_CONNECTOR=/tmp/virtq ./build/app/testpmd -c f -n 4 -m 1024 --use-device \
          'eth_virtq0;mac=00:01:02:03:04:05;lcore_id=1,eth_pcap0;iface=eth0' -- -i

 User can specify following options for virtq PMD.
 - `RTE_PMD_VIRTQ_CONNECTOR` : This has to be the same parameter as "socketpath" of QEMU.
 - `eth_virtqX` : Specify virtual device is for "virtq PMD". The suffix number "X" of eth_virtqX must be started from 0 and incremented by one. "X" corresponds to "nid" provided by QEMU.
 - `mac` : Mac address of the device virtualized by virtq PMD.
 - `lcore_id` : Core id of event handler thread created by virtq PMD. This is optional.

 Internally, virtq PMD creates a thread to handle events come from unix domain socket.


- Stop QEMU

 Type 'shutdown' command on the guest. Or users can send a kill signal to QEMU.

- Stop a DPDK application on the guest

 Type 'stop', 'stop port all' and 'quit' using command line of testpmd. User should stop port before quit. Without this, virtio-net PMD will not finalize virtio-net-ipc device. It means virtio-net-ipc device cannot notice it to virtq PMD. As a result, virtq PMD may access memory allocated by QEMU, even though it has been already freed. In the case, see later sections to recover.

- Stop a DPDK application on the host

 Type 'stop', 'stop port all' and 'quit' using command line of testpmd.

###How to recover when QEMU or DPDK applications on the host or the guest is gone  suddenly or without finalization###

- Case1: A DPDK application on the guest is gone

 Before restarting the application, users should finalize virtio-net-ipc device. To do that, type followings.

        $ ./tools/pci_unbind -b virtio-pci <pci address of virtio-net-ipc device>
        $ ./tools/pci_unbind -b igb_uio <pci address of virtio-net-ipc device>

 Above commands will let the kernel virtio-net driver handle the device once, then bind to DPDK again. When the kernel driver binds the device, virtio-net-ipc device will be finalized and initialized.


- Case2: QEMU is gone

 Just wait a few seconds. virtq PMDs are checking unix domain socket. When QEMU is gone, virtq PMD can know it by broken pipe. An only risk is the case QEMU or DPDK application that uses hugetlbfs starts before virtq PMD doesn't receive broken pipe. In the case, QEMU and the application can re-use memory again. At the same time virtq PMD will read or write that memory. To prevent that, just give few seconds to virtq PMD.


- Case3: A DPDK application on the host is gone

 Nothing to do. Just restart the application again.

###How to work with lagopus###

If you want to execute lagopus with virtq PMD, you have to compile lagopus with virtq-pmd-dpdk. 
Instruction is following:

- Download

 - lagopus

            $ git clone https://github.com/lagopus/lagopus.git
            $ cd lagopus
            $ git checkout -b 0.1.1 refs/tags/v0.1.1

- Apply patch to lagopus 0.1.1
 - patch file "lagopus.patch" is the following.
```diff
    --- lagopus/configure.ac	2015-01-28 00:39:47.713068317 +0900
    +++ configure.ac	2015-01-28 22:17:56.377004617 +0900
    @@ -253,6 +253,8 @@
                     RTE_LIBS="$RTE_LIBS -Xlinker -lrte_pmd_ring",)
       AC_CHECK_FILE(${RTE_SDK}/${RTE_TARGET}/lib/librte_pmd_pcap.a,
                     RTE_LIBS="$RTE_LIBS -Xlinker -lrte_pmd_pcap",)
    +  AC_CHECK_FILE(${RTE_SDK}/${RTE_TARGET}/lib/librte_pmd_virtq.a,
    +                RTE_LIBS="$RTE_LIBS -Xlinker -lrte_pmd_virtq",)
       RTE_LIBS="$RTE_LIBS -Xlinker --end-group -Xlinker --no-whole-archive"
     else
      RTE_SDK=
```

 - Apply patch to configure.ac

            $ patch < lagopus.patch

 - re-generate configure

            $ aclocal
            $ autoheader
            $ automake
            $ autoconf

- Compile

   here is in lagopus directory, and you have to set "RTE_SDK" environment variable to "virtq-pmd-dpdk" directory. And then type the following command.

            $ ./configure --with-dpdk-dir=${RTE_SDK}
            $ make
            $ sudo make install

 More details of options, see [lagopus documentations](https://github.com/lagopus/lagopus/blob/master/QUICKSTART.md).

###How to invoke virtq PMD from lagopus###

- Here is an example which provides two virtual nic attached to lagopus. In this example, two physical nics are attached to DPDK igb_uio kernel module.

            $ sudo RTE_PMD_VIRTQ_CONNECTOR=/tmp/virtq lagopus -d -- -c3 -n1 -m1024 --use-device 'eth_virtq0,eth_virtq1' -- -pf
```json
interface {
    ethernet {
        eth0;
        eth1;
        eth2;
        eth3;
    }
}
bridge-domains {
    br0 {
	dpid 0.00:00:00:00:00:01;
        port {
            eth0;
            eth1;
            eth2;
            eth3;
        }
        controller {
            127.0.0.1;
        }
    }
}
```


