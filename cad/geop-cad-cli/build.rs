//! Gives a Windows build of the double-clicked program its icon: Explorer
//! shows the one in the file, which nothing later can add.

fn main() {
    println!("cargo:rerun-if-changed=assets/geop.ico");
    #[cfg(windows)]
    if std::env::var_os("CARGO_FEATURE_DESKTOP").is_some() {
        let mut resource = winresource::WindowsResource::new();
        resource.set_icon("assets/geop.ico");
        resource.compile().expect("the icon is embedded");
    }
}
