use std::path::PathBuf;
use std::{env, fs};

use ch32_metapac::metadata::METADATA;
use proc_macro2::TokenStream;
use quote::{format_ident, quote};

fn main() {
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    // TODO: generate singletons

    let mut g = TokenStream::new();

    // ========
    // Extract the rcc registers
    // let rcc_registers = METADATA
    //     .peripherals
    //     .iter()
    //     .filter_map(|p| p.registers.as_ref())
    //     .find(|r| r.kind == "rcc")
    //     .unwrap();

    for p in METADATA.peripherals {
        //if !singletons.contains(&p.name.to_string()) {
        //    continue;
        //}
        let pname = format_ident!("{}", p.name);

        if let Some(rcc) = &p.rcc {
            let en = rcc.enable.as_ref().unwrap();

            let rst = match &rcc.reset {
                Some(rst) => {
                    let rst_reg = format_ident!("{}", rst.register.to_ascii_lowercase());
                    let set_rst_field = format_ident!("set_{}", rst.field.to_ascii_lowercase());
                    quote! {
                        crate::pac::RCC.#rst_reg().modify(|w| w.#set_rst_field(true));
                        crate::pac::RCC.#rst_reg().modify(|w| w.#set_rst_field(false));
                    }
                }
                None => TokenStream::new(),
            };

            // let ptype = if let Some(reg) = &p.registers { reg.kind } else { "" };

            let en_reg = format_ident!("{}", en.register.to_ascii_lowercase());
            let set_en_field = format_ident!("set_{}", en.field.to_ascii_lowercase());

            g.extend(quote! {
                impl crate::peripheral::sealed::RccPeripheral for peripherals::#pname {
                    fn enable_and_reset_with_cs(_cs: critical_section::CriticalSection) {
                        crate::pac::RCC.#en_reg().modify(|w| w.#set_en_field(true));
                        #rst
                    }
                    fn disable_with_cs(_cs: critical_section::CriticalSection) {
                        crate::pac::RCC.#en_reg().modify(|w| w.#set_en_field(false));
                    }
                }

                impl crate::peripheral::RccPeripheral for peripherals::#pname {}
            });
        }

        if let Some(remap) = &p.remap {
            let remap_reg = format_ident!("{}", remap.register.to_ascii_lowercase());
            let set_remap_field = format_ident!("set_{}", remap.field.to_ascii_lowercase());

            g.extend(quote! {
                impl crate::peripheral::sealed::RemapPeripheral for peripherals::#pname {
                    fn set_remap(remap: u8) {
                        crate::pac::AFIO.#remap_reg().modify(|w| w.#set_remap_field(remap));
                    }
                }

                impl crate::peripheral::RemapPeripheral for peripherals::#pname {}
            });
        }
    }

    // ========
    // Write generated.rs

    let out_file = out_dir.join("_generated.rs").to_string_lossy().to_string();
    fs::write(out_file, g.to_string()).unwrap();

    // Put the linker script somewhere the linker can find it.
    fs::write(out_dir.join("memory.x"), include_bytes!("memory.x")).unwrap();
    println!("cargo:rustc-link-search={}", out_dir.display());
    println!("cargo:rerun-if-changed=memory.x");

    println!("cargo:rerun-if-changed=build.rs");
}
