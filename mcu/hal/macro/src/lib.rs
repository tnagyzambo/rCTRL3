#![no_std]

use proc_macro::TokenStream;
use quote::{format_ident, quote};
use syn::{parse_macro_input, DataEnum, DeriveInput, Fields, Ident};

// TODO: Clean this up and make it more robust
// This is an experiment in using proc macros to slightly
// abstract the PAC and provide some iterators over the read registers
// This is particularly useful for interrupt registers that have all bits
// cleared on read.
// Deriving these features on an enum with defined discriminants gives a clean way to iterrate over the
// the bits in the register

#[proc_macro_derive(TryFromU8)]
pub fn try_from_u8_derive(input: TokenStream) -> TokenStream {
    let DeriveInput { ident, data, .. } = parse_macro_input!(input as DeriveInput);

    match data {
        syn::Data::Enum(data) => try_from_u8_impl(ident, data),
        _ => panic!("Can only derive TryFromU8 for enum"),
    }
}

fn try_from_u8_impl(enum_ident: Ident, data: DataEnum) -> TokenStream {
    let match_arms = data.variants.iter().filter_map(|v| match v.fields {
        Fields::Unit => {
            let variant_ident = &v.ident;
            let variant_value = match &v.discriminant {
                Some(d) => d.1.clone(),
                None => panic!("No enum discriminant set"),
            };
            Some(quote! {#variant_value => Ok(#enum_ident::#variant_ident),})
        }
        _ => None,
    });

    quote! {
        impl TryFrom<u8> for #enum_ident {
                type Error = &'static str;

                fn try_from(value: u8) -> Result<Self, Self::Error> {
                    match value {
                        #(#match_arms)*
                        _ => Err("Invalid enumeration"),
                    }
                }
            }
    }
    .into()
}

#[proc_macro_derive(BitFieldIterator)]
pub fn bit_field_iterator_derive(input: TokenStream) -> TokenStream {
    let DeriveInput { ident, data, .. } = parse_macro_input!(input as DeriveInput);

    match data {
        syn::Data::Enum(data) => bit_field_iterator_impl(ident, data),
        _ => panic!("Can only derive BitFieldIterator for enum"),
    }
}

fn bit_field_iterator_impl(enum_ident: Ident, _data: DataEnum) -> TokenStream {
    let struct_ident = format_ident!("{}Iterator", enum_ident);

    quote! {
        pub struct #struct_ident {
            reg: u32,
            index: u8,
        }

        impl #struct_ident {
            pub fn new(reg: u32) -> Self {
                Self { reg, index: 0 }
            }
        }

        impl Iterator for #struct_ident {
            type Item = #enum_ident;

            fn next(&mut self) -> Option<Self::Item> {
                if self.index > 31 {
                    return None
                }

                let mask = self.reg & (1 << self.index);
                if mask != 0 {
                    if let Ok(item) = #enum_ident::try_from(self.index) {
                        self.index += 1;
                        return Some(item)
                    }
                }
                self.index += 1;
                return self.next()
            }
        }
    }
    .into()
}
