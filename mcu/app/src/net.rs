use hal::gmac::{mac_state, Mac, PhyInterrupt};
use hal::rtt::Rtt;

pub type NetworkReference = smoltcp_nal::shared::NetworkStackProxy<'static, NetworkStack>;

pub type NetworkStack = smoltcp_nal::NetworkStack<'static, Mac<mac_state::Enabled>, Rtt>;

pub type NetworkManager =
    smoltcp_nal::shared::NetworkManager<'static, Mac<mac_state::Enabled>, Rtt>;

pub struct NetworkProcessor {
    stack: NetworkReference,
}

impl NetworkProcessor {
    fn new(stack: NetworkReference) -> Self {
        Self { stack }
    }

    pub fn handle_mdio_interrupt(&mut self) {
        let mac = self.stack.lock(|stack| stack.interface_mut().device_mut());
        for interrupt in mac.mdio_interrupts() {
            match interrupt {
                PhyInterrupt::LinkDown => defmt::info!("LINK DOWN"),
                PhyInterrupt::LinkUp => {
                    defmt::info!("LINK UP");
                }
                _ => (),
            }
        }
    }
}

pub struct NetworkComponents {
    pub processor: NetworkProcessor,
}

impl NetworkComponents {
    pub fn new(stack_manger: &'static mut NetworkManager) -> Self {
        let processor = NetworkProcessor::new(stack_manger.acquire_stack());

        Self { processor }
    }
}
