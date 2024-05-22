#[cfg(target_arch = "x86_64")]
use crate::drivers::interrupt::register_handler;
use crate::mm::phys_to_virt;
use crate::percpu::PerCpu;
use core::ptr::{read_volatile, write_volatile};
use x86::io::{inb, inl, inw, outb, outl, outw};

const PCI_ID_ANY: u32 = 0xffff;
const PCI_CFG_VENDOR_ID: u16 = 0x000;
const PCI_CFG_DEVICE_ID: u16 = 0x002;
const PCI_CFG_COMMAND: u16 = 0x004;
const PCI_CMD_MEM: u16 = 1 << 1;
const PCI_CMD_MASTER: u16 = 1 << 2;
const PCI_CFG_STATUS: u16 = 0x006;
const PCI_STS_CAPS: u16 = 1 << 4;
const PCI_CFG_BAR: u16 = 0x010;
const PCI_BAR_64BIT: u16 = 0x4;
const PCI_CFG_CAP_PTR: u32 = 0x34;
const PCI_CONE: u32 = 1 << 31;
const PCI_REG_ADDR_PORT: u16 = 0xcf8;
const PCI_REG_DATA_PORT: u16 = 0xcfc;
const PCI_CAP_MSI: u16 = 0x05;
const PCI_CAP_MSIX: u16 = 0x11;
const MSIX_CTRL_ENABLE: u32 = 0x8000;
const MSIX_CTRL_FMASK: u32 = 0x4000;
const HDA_GCTL: u32 = 0x08;
const HDA_WAKEEN: u32 = 0x0c;
const HDA_STATESTS: u32 = 0x0e;
const HDA_INTCTL: u32 = 0x20;
const IRQ_VECTOR: u32 = 32;

pub static mut HDBAR: *mut u64 = core::ptr::null_mut();

unsafe fn mmio_read16(address: *const u16) -> u16 {
    read_volatile(address)
}

unsafe fn mmio_read32(address: *const u32) -> u32 {
    read_volatile(address)
}

unsafe fn mmio_write16(address: *mut u16, value: u16) {
    write_volatile(address, value);
}

unsafe fn mmio_write32(address: *mut u32, value: u32) {
    write_volatile(address, value);
}

unsafe fn pci_read_config(bdf: u16, addr: u32, size: u32) -> u32 {
    // select pci device
    outl(
        PCI_REG_ADDR_PORT,
        PCI_CONE | ((bdf as u32) << 8) | (addr & 0xfc),
    );
    match size {
        1 => inb(PCI_REG_DATA_PORT + (addr & 0x3) as u16) as u32,
        2 => inw(PCI_REG_DATA_PORT + (addr & 0x3) as u16) as u32,
        4 => inl(PCI_REG_DATA_PORT),
        _ => u32::MAX,
    }
}

unsafe fn pci_write_config(bdf: u16, addr: u32, value: u32, size: u32) {
    // select pci device
    outl(
        PCI_REG_ADDR_PORT,
        PCI_CONE | ((bdf as u32) << 8) | (addr & 0xfc),
    );
    match size {
        1 => outb(PCI_REG_DATA_PORT + (addr & 0x3) as u16, value as u8 & 0xff),
        2 => outw(
            PCI_REG_DATA_PORT + (addr & 0x3) as u16,
            value as u16 & 0xffff,
        ),
        4 => outl(PCI_REG_DATA_PORT, value),
        _ => (),
    }
}

fn pci_find_device(vendor: u16, device: u16, start_bdf: u16) -> i32 {
    for bdf in start_bdf..u16::MAX {
        let id = unsafe { pci_read_config(bdf, PCI_CFG_VENDOR_ID as u32, 2) };
        if id == PCI_ID_ANY || (vendor != PCI_ID_ANY as u16 && vendor as u32 != id) {
            continue;
        }
        if device == PCI_ID_ANY as u16
            || unsafe { pci_read_config(bdf, PCI_CFG_DEVICE_ID as u32, 2) } == device as u32
        {
            debug!("this is return bdf: {:#x}", bdf);
            return bdf as i32;
        }
    }
    -1
}

unsafe fn pci_find_cap(bdf: u16, cap: u16) -> i32 {
    let mut pos = PCI_CFG_CAP_PTR - 1;

    if pci_read_config(bdf, PCI_CFG_STATUS as u32, 2) & PCI_STS_CAPS as u32 == 0 {
        return -1;
    }

    loop {
        pos = pci_read_config(bdf, pos + 1, 1);
        if pos == 0 {
            return -1;
        }
        if pci_read_config(bdf, pos, 1) as u16 == cap {
            return pos as i32;
        }
    }
}

// unsafe fn pci_msix_set_vector(bdf: u16, vector: u32, index: u32) {
//     let cap = pci_find_cap(bdf, PCI_CAP_MSIX);
//     if cap < 0 {
//         return;
//     }
//     let cap = cap as u32;
//     let mut ctrl = pci_read_config(bdf, cap + 2, 2);
//     if index > (ctrl & 0x3ff) as u32 {
//         return;
//     }
//     let table = pci_read_config(bdf, cap + 4, 4);
//     let bar = (table & 7) * 4 + PCI_CFG_BAR as u32;
//     let addr = pci_read_config(bdf, bar, 4);
//     let mut msix_table: u64 = 0;
//     if (addr & 6) == PCI_BAR_64BIT as u32 {
//         msix_table = pci_read_config(bdf, bar + 4, 4) as u64;
//         msix_table <<= 32;
//     }
//     msix_table |= (addr & !0xf) as u64;
//     msix_table += (table & !7) as u64;

//     ctrl |= (MSIX_CTRL_ENABLE | MSIX_CTRL_FMASK) as u32;
//     pci_write_config(bdf, cap + 2, ctrl, 2);

//     msix_table += 16 * index as u64;
//     mmio_write32(
//         msix_table as *mut u32,
//         0xfee00000 | (PerCpu::current_cpu_id() << 12) as u32,
//     );
//     mmio_write32((msix_table + 4) as *mut u32, 0);
//     mmio_write32((msix_table + 8) as *mut u32, vector);
//     mmio_write32((msix_table + 12) as *mut u32, 0);

//     ctrl &= !(MSIX_CTRL_FMASK as u32);
//     pci_write_config(bdf, cap + 2, ctrl, 2);
// }

unsafe fn pci_msi_set_vector(bdf: u16, vector: u32) {
    println!("pci_msi_set_vector");
    let cap = pci_find_cap(bdf, PCI_CAP_MSI);
    if cap < 0 {
        return;
    }
    let cap = cap as u32;
    pci_write_config(
        bdf,
        cap + 0x04,
        0xfee00000 | (PerCpu::current_cpu_id() << 12) as u32,
        4,
    );
    let ctl = pci_read_config(bdf, cap + 0x02, 2);
    let data: u32;
    if ctl & (1 << 7) != 0 {
        pci_write_config(bdf, cap + 0x08, 0, 4);
        data = cap + 0x0c;
    } else {
        data = cap + 0x08;
    }
    pci_write_config(bdf, data, vector, 2);
    pci_write_config(bdf, cap + 0x02, 0x0001, 2);
}

unsafe fn pci_msix_init(bdf: u16, vector: u32) {
    println!("pci_msix_init");
    let cap = pci_find_cap(bdf, PCI_CAP_MSIX);
    if cap < 0 {
        return;
    }
    let cap = cap as u32;
    let msix_table_offset = pci_read_config(bdf, cap + 0x04, 4) & 0xfffffff8;
    let msix_pba_offset = pci_read_config(bdf, cap + 0x08, 4) & 0xfffffff8;
    let msix_entries = (pci_read_config(bdf, cap + 0x02, 2) & 0x07ff) + 1; // message control: The number of table entries is the <value read> + 1.

    let bar_msix_idx = PCI_CFG_BAR as u32 + 0x4; // 0x14 BAR1
    let bar_msix = pci_read_config(bdf, bar_msix_idx, 4);
    println!(
        "msix_table_offset: {:#x} msix_pba_offset: {:#x} bar msix:{:#x}",
        msix_table_offset, msix_pba_offset, bar_msix,
    );

    for i in 0..msix_entries {
        let base = phys_to_virt((msix_table_offset + i * 16 + bar_msix) as usize);
        println!(
            "write msix entry: base: {:#x} write msg addr:{:#x}",
            base,
            0xfee00000 | (1 << 12) as u32
        );
        let current_cpu = PerCpu::current_cpu_id();
        mmio_write32(base as *mut u32, 0xfee00000 | (current_cpu << 12) as u32);
        println!("write msix entry: base: {:#x} write upper msg addr 0", base);
        mmio_write32((base + 0x04) as *mut u32, 0);
        println!(
            "write msix entry: base: {:#x} write msg data: {:#x}",
            base, vector
        );
        mmio_write32((base + 0x08) as *mut u32, vector);
        println!("write msix entry: base: {:#x} vector control 0", base);
        mmio_write32((base + 0x0c) as *mut u32, 0);

        // let msg_addr =  mmio_read32(base as *mut u32);
        // let msg_upper_addr =  mmio_read32((base + 0x04) as *mut u32);
        // let msg_data =  mmio_read32((base + 0x08) as *mut u32);
        // let msg_control =  mmio_read32((base + 0x0c) as *mut u32);
        // println!("read msix entry: msg_addr: {:#x} msg_upper_addr: {:#x} msg_data: {:#x} msg_control: {:#x}", msg_addr, msg_upper_addr, msg_data, msg_control);
    }

    let ctl = pci_read_config(bdf, cap + 0x2, 2);
    pci_write_config(bdf, cap + 0x2, ctl | 0x8000, 2);
}

fn irq_handler() {
    unsafe {
        let addr_u16 = phys_to_virt(HDBAR as usize) as *mut u16;
        let statests = read_volatile(addr_u16.offset(HDA_STATESTS as isize));
        println!(
            "[pci irq handler] HDA MSI received (STATESTS: {:04x})",
            statests
        );
    }
}

pub fn pci_init() {
    let mut bar: u64;
    let bdf: i32;

    println!("this is pci init");
    register_handler(IRQ_VECTOR as usize, irq_handler);

    bdf = pci_find_device(PCI_ID_ANY as u16, PCI_ID_ANY as u16, 0);
    if bdf < 0 {
        println!("No device found!");
        return;
    }
    let bdf = bdf as u16;
    unsafe {
        println!(
            "Found {:04x}:{:04x} at {:02x}:{:02x}.{:x}",
            pci_read_config(bdf, PCI_CFG_VENDOR_ID as u32, 2),
            pci_read_config(bdf, PCI_CFG_DEVICE_ID as u32, 2),
            bdf >> 8,
            (bdf >> 3) & 0x1f,
            bdf & 0x3
        );

        println!("before read bar");
        // read 0 bar addr
        bar = pci_read_config(bdf, PCI_CFG_BAR as u32, 4) as u64;
        println!("this is bar origin addr: {:#x}", bar);
        // if bit 2 is 1, 64 bit addr space
        if (bar & 0b110) == 0b100 {
            bar |= (pci_read_config(bdf, PCI_CFG_BAR as u32 + 4, 4) as u64) << 32;
        }
        // 0:3 bit is control bit
        HDBAR = (bar & !0xf) as *mut u64;

        println!("HDBAR: {:#x}", HDBAR as u64);

        pci_msix_init(bdf, IRQ_VECTOR);
        // pci_msi_set_vector(bdf, IRQ_VECTOR);

        pci_write_config(
            bdf,
            PCI_CFG_COMMAND as u32,
            (PCI_CMD_MEM | PCI_CMD_MASTER) as u32,
            2,
        );
    }
}

pub fn pci_main() {
    println!("this is pci main");

    let bdf = pci_find_device(PCI_ID_ANY as u16, PCI_ID_ANY as u16, 0);
    if bdf < 0 {
        println!("No device found!");
        return;
    }
    let bdf = bdf as u16;

    unsafe {
        let addr_u16 = phys_to_virt(HDBAR as usize) as *mut u16;
        // let addr_u16 = HDBAR as *mut u16;
        let ptr = addr_u16.offset(HDA_STATESTS as isize);
        debug!("ptr:{:#x}", ptr as usize);
        let value = mmio_read16(addr_u16.offset(HDA_STATESTS as isize));
        println!("HDA_STATESTS: {:#x}", value);
        mmio_write16(addr_u16.offset(HDA_STATESTS as isize), value);

        mmio_write16(addr_u16.offset(HDA_GCTL as isize), 0);
        mmio_write16(addr_u16.offset(HDA_GCTL as isize), 1);

        mmio_write16(addr_u16.offset(HDA_WAKEEN as isize), 0x0f);
        let value = mmio_read16(addr_u16.offset(HDA_WAKEEN as isize));
        println!("after write HDA_WAKEEN write value: {:#x}", value);

        let addr_u32 = phys_to_virt(HDBAR as usize) as *mut u32;
        mmio_write32(addr_u32.offset(HDA_INTCTL as isize), (1 << 31) | (1 << 30));

        pci_write_config(bdf, 0xd as u32, 0, 1);
    }
}
