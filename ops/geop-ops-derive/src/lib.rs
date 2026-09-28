//! Derives that describe `geop_ops` operations to anything that edits
//! programs — a UI generates its forms from these descriptions instead of
//! knowing each operation by hand.
//!
//! - `#[derive(OperationArgs)]` on an operation's arguments struct: every
//!   field carries `#[arg(<ArgKind>)]` saying what kind of value it holds
//!   (and so how it is entered), and its doc comment becomes its
//!   description.
//! - `#[derive(Operations)]` on an enum of operations, implementing
//!   `geop_ops::operation::Operations`: every variant `Name(NameArgs)`
//!   dispatches to the unit struct `Name`, which implements `Operation`; its
//!   doc comment describes the operation and `#[operation(label = "...")]`
//!   gives its short name.

use proc_macro::TokenStream;
use quote::quote;
use syn::{
    Attribute, Data, DeriveInput, Expr, Fields, LitStr, Meta, parse_macro_input, spanned::Spanned,
};

/// The text of `attrs`' doc comments, one line each, with the space
/// rustdoc puts after `///` removed.
fn doc_of(attrs: &[Attribute]) -> String {
    attrs
        .iter()
        .filter_map(|a| match &a.meta {
            Meta::NameValue(nv) if nv.path.is_ident("doc") => match &nv.value {
                Expr::Lit(syn::ExprLit {
                    lit: syn::Lit::Str(s),
                    ..
                }) => Some(s.value()),
                _ => None,
            },
            _ => None,
        })
        .map(|line| line.strip_prefix(' ').unwrap_or(&line).to_string())
        .collect::<Vec<_>>()
        .join("\n")
}

/// `Number { .. }` as `::geop_ops::operation::ArgKind::Number { .. }`:
/// an argument names a kind by its bare variant, and it must mean that
/// variant whatever else the module has in scope under the same name.
fn qualify(kind: Expr) -> syn::Result<Expr> {
    let prefix: syn::Path = syn::parse_quote!(::geop_ops::operation::ArgKind);
    let qualified = |path: &syn::Path| -> syn::Result<syn::Path> {
        let Some(variant) = path.get_ident() else {
            return Err(syn::Error::new(
                path.span(),
                "expected an `ArgKind` variant, e.g. `Solid`",
            ));
        };
        let mut full = prefix.clone();
        full.segments.push(variant.clone().into());
        Ok(full)
    };
    Ok(match kind {
        Expr::Path(mut p) => {
            p.path = qualified(&p.path)?;
            Expr::Path(p)
        }
        Expr::Struct(mut s) => {
            s.path = qualified(&s.path)?;
            Expr::Struct(s)
        }
        other => {
            return Err(syn::Error::new(
                other.span(),
                "expected an `ArgKind` variant, e.g. `Solid` or `Number { .. }`",
            ));
        }
    })
}

/// `impl OperationArgs`: the schema of every field, from its
/// `#[arg(<ArgKind expression>)]` and its doc comment.
#[proc_macro_derive(OperationArgs, attributes(arg))]
pub fn derive_operation_args(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let Data::Struct(data) = &input.data else {
        return syn::Error::new(input.span(), "OperationArgs needs a struct")
            .to_compile_error()
            .into();
    };
    let Fields::Named(fields) = &data.fields else {
        return syn::Error::new(input.span(), "OperationArgs needs named fields")
            .to_compile_error()
            .into();
    };
    let mut schemas = Vec::new();
    for field in &fields.named {
        let ident = field.ident.as_ref().expect("named field");
        let Some(attr) = field.attrs.iter().find(|a| a.path().is_ident("arg")) else {
            return syn::Error::new(
                field.span(),
                "every argument needs #[arg(<ArgKind>)] saying what kind of value it holds",
            )
            .to_compile_error()
            .into();
        };
        let kind: Expr = match attr.parse_args() {
            Ok(kind) => kind,
            Err(e) => return e.to_compile_error().into(),
        };
        let kind = match qualify(kind) {
            Ok(kind) => kind,
            Err(e) => return e.to_compile_error().into(),
        };
        let field_name = ident.to_string();
        let doc = doc_of(&field.attrs);
        schemas.push(quote! {
            ::geop_ops::operation::ArgSchema {
                name: #field_name,
                doc: #doc,
                kind: #kind,
            }
        });
    }
    quote! {
        impl ::geop_ops::operation::OperationArgs for #name {
            fn schema() -> ::std::vec::Vec<::geop_ops::operation::ArgSchema> {
                ::std::vec![#(#schemas),*]
            }
        }
    }
    .into()
}

/// `PascalCase` to `snake_case`, the way serde's `rename_all` does it.
fn snake_case(name: &str) -> String {
    let mut out = String::new();
    for (i, c) in name.chars().enumerate() {
        if c.is_uppercase() {
            if i > 0 {
                out.push('_');
            }
            out.extend(c.to_lowercase());
        } else {
            out.push(c);
        }
    }
    out
}

/// Dispatch, conversions and schemas for the enum of every operation.
#[proc_macro_derive(Operations, attributes(operation))]
pub fn derive_operations(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let Data::Enum(data) = &input.data else {
        return syn::Error::new(input.span(), "Operations needs an enum")
            .to_compile_error()
            .into();
    };
    let mut apply_arms = Vec::new();
    let mut dialog_arms = Vec::new();
    let mut handle_arms = Vec::new();
    let mut kind_arms = Vec::new();
    let mut label_arms = Vec::new();
    let mut schemas = Vec::new();
    let mut froms = Vec::new();
    for variant in &data.variants {
        let op = &variant.ident;
        let Fields::Unnamed(fields) = &variant.fields else {
            return syn::Error::new(variant.span(), "every operation is `Name(NameArgs)`")
                .to_compile_error()
                .into();
        };
        let Some(args) = fields.unnamed.first().map(|f| &f.ty) else {
            return syn::Error::new(variant.span(), "every operation is `Name(NameArgs)`")
                .to_compile_error()
                .into();
        };
        let mut label = op.to_string();
        for attr in variant
            .attrs
            .iter()
            .filter(|a| a.path().is_ident("operation"))
        {
            let parsed = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("label") {
                    label = meta.value()?.parse::<LitStr>()?.value();
                    Ok(())
                } else {
                    Err(meta.error("expected `label = \"...\"`"))
                }
            });
            if let Err(e) = parsed {
                return e.to_compile_error().into();
            }
        }
        let kind = snake_case(&op.to_string());
        let doc = doc_of(&variant.attrs);
        apply_arms.push(quote! {
            #name::#op(args) => ::geop_ops::operation::Operation::<S>::apply(
                &#op, part, operation_id, args,
            ),
        });
        dialog_arms.push(quote! {
            #name::#op(args) => ::geop_ops::operation::Operation::<S>::dialog(&#op, before, args),
        });
        handle_arms.push(quote! {
            #name::#op(args) => ::geop_ops::operation::Operation::<S>::handles(&#op, before, args),
        });
        kind_arms.push(quote! { #name::#op(_) => #kind, });
        label_arms.push(quote! { #name::#op(_) => #label, });
        schemas.push(quote! {
            ::geop_ops::operation::OperationSchema {
                kind: #kind,
                label: #label,
                doc: #doc,
                args: <#args as ::geop_ops::operation::OperationArgs>::schema(),
            }
        });
        froms.push(quote! {
            impl ::std::convert::From<#args> for #name {
                fn from(args: #args) -> Self {
                    #name::#op(args)
                }
            }
        });
    }
    quote! {
        impl ::geop_ops::operation::Operations for #name {
            fn apply<S: ::geop_core_math::scalars::Scalar>(
                &self,
                part: ::geop_ops::Part<S>,
                operation_id: &str,
            ) -> ::geop_core_math::geop_error::GeopResult<::geop_ops::Part<S>> {
                match self {
                    #(#apply_arms)*
                }
            }

            fn dialog<S: ::geop_core_math::scalars::Scalar>(
                &self,
                before: &::geop_ops::Part<S>,
            ) -> ::geop_ops::operation::Dialog {
                match self {
                    #(#dialog_arms)*
                }
            }

            fn handles<S: ::geop_core_math::scalars::Scalar>(
                &self,
                before: &::geop_ops::Part<S>,
            ) -> ::geop_core_math::geop_error::GeopResult<
                ::std::vec::Vec<::geop_ops::operation::Handle>,
            > {
                match self {
                    #(#handle_arms)*
                }
            }

            fn kind(&self) -> &'static str {
                match self {
                    #(#kind_arms)*
                }
            }

            fn label(&self) -> &'static str {
                match self {
                    #(#label_arms)*
                }
            }

            fn schemas() -> ::std::vec::Vec<::geop_ops::operation::OperationSchema> {
                ::std::vec![#(#schemas),*]
            }
        }
        #(#froms)*
    }
    .into()
}
