mod gui;

// We run the gui inside of a tokio runtime
#[tokio::main]
async fn main() -> Result<(), eframe::Error> {
    let options = eframe::NativeOptions::default();

    eframe::run_native("rCTRL", options, Box::new(|cc| Box::new(gui::App::new(cc))))
}
