use proc_macro::TokenStream;
use quote::{format_ident, quote};
use std::collections::HashSet;
use syn::{
    parse::{Parse, ParseStream},
    parse_macro_input,
    punctuated::Punctuated,
    DataEnum, DeriveInput, Fields, Ident, Result, Token, Type,
};

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

// TODO: Clean this up
// proc_macro to implement PinId for SAMV71 based on a list of their designators

#[proc_macro]
pub fn pins(tokens: TokenStream) -> TokenStream {
    // Parse input into a list of arguements
    let input = parse_macro_input!(tokens as Args);
    let mut output = TokenStream::new();

    for pin in input.pins {
        let pin_string = pin.to_string();
        let (bank, id) = pin_string.split_at(2);
        let reg = Type::Verbatim(
            format!("PIO{}::PTR", bank.chars().nth(1).unwrap())
                .parse()
                .unwrap(),
        );
        let id = id.parse::<u8>().unwrap();

        output.extend::<TokenStream>(
            quote! {
                pub struct #pin {}

                impl PinId for #pin {
                    const REG: *const RegisterBlock = #reg;
                    const ID: u8 = #id;
                }
            }
            .into(),
        )
    }

    output.into()
}

struct Args {
    pins: HashSet<Ident>,
}

impl Parse for Args {
    fn parse(input: ParseStream) -> Result<Self> {
        let pins = Punctuated::<Ident, Token![,]>::parse_terminated(input as ParseStream)?;

        Ok(Args {
            pins: pins.into_iter().collect(),
        })
    }
}
