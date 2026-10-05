//! `#[derive(Operations)]`: a set of `geop_ops` operations as one enum.
//!
//! On an enum whose every variant is `Name(NameArgs)`, with `Name` a unit
//! struct implementing `geop_ops::Operation` with `Args = NameArgs`, it
//! implements `geop_ops::Operations` — dispatching every method to `Name` —
//! and `From<NameArgs>` for the enum. A variant's doc comment describes the
//! operation; `#[operation(...)]` gives its short name if that is not the
//! variant's (`label = "..."`), and the group an editor files it in and how
//! prominently, which every operation names (`group = Features, tier = Big`:
//! an `OperationGroup` and an `OperationTier`).

use proc_macro::TokenStream;
use quote::quote;
use syn::{
    Attribute, Data, DeriveInput, Expr, Fields, Ident, LitStr, Meta, parse_macro_input,
    spanned::Spanned,
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

/// `impl Operations` and the conversions for the enum of a set of
/// operations.
#[proc_macro_derive(Operations, attributes(operation))]
pub fn derive_operations(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let Data::Enum(data) = &input.data else {
        return syn::Error::new(input.span(), "Operations needs an enum")
            .to_compile_error()
            .into();
    };
    let mut infos = Vec::new();
    let mut new_arms = Vec::new();
    let mut apply_arms = Vec::new();
    let mut picks_built_arms = Vec::new();
    let mut session_arms = Vec::new();
    let mut form_arms = Vec::new();
    let mut set_arms = Vec::new();
    let mut event_arms = Vec::new();
    let mut formulas_arms = Vec::new();
    let mut kind_arms = Vec::new();
    let mut label_arms = Vec::new();
    let mut froms = Vec::new();
    for variant in &data.variants {
        let op = &variant.ident;
        let Some(args) = (match &variant.fields {
            Fields::Unnamed(fields) if fields.unnamed.len() == 1 => {
                fields.unnamed.first().map(|f| &f.ty)
            }
            _ => None,
        }) else {
            return syn::Error::new(variant.span(), "every operation is `Name(NameArgs)`")
                .to_compile_error()
                .into();
        };
        let mut label = op.to_string();
        let mut group: Option<Ident> = None;
        let mut tier: Option<Ident> = None;
        for attr in variant
            .attrs
            .iter()
            .filter(|a| a.path().is_ident("operation"))
        {
            let parsed = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("label") {
                    label = meta.value()?.parse::<LitStr>()?.value();
                    Ok(())
                } else if meta.path.is_ident("group") {
                    group = Some(meta.value()?.parse::<Ident>()?);
                    Ok(())
                } else if meta.path.is_ident("tier") {
                    tier = Some(meta.value()?.parse::<Ident>()?);
                    Ok(())
                } else {
                    Err(meta.error(
                        "expected `label = \"...\"`, `group = <OperationGroup>` or `tier = <OperationTier>`",
                    ))
                }
            });
            if let Err(e) = parsed {
                return e.to_compile_error().into();
            }
        }
        let (Some(group), Some(tier)) = (group, tier) else {
            return syn::Error::new(
                variant.span(),
                "every operation names its group and tier: \
                 `#[operation(group = <OperationGroup>, tier = <OperationTier>)]`",
            )
            .to_compile_error()
            .into();
        };
        let kind = snake_case(&op.to_string());
        let doc = doc_of(&variant.attrs);
        let operation = quote!(::geop_ops::operation::Operation);
        infos.push(quote! {
            ::geop_ops::operation::OperationInfo {
                kind: #kind,
                label: #label,
                doc: #doc,
                group: ::geop_ops::operation::OperationGroup::#group,
                tier: ::geop_ops::operation::OperationTier::#tier,
            }
        });
        new_arms.push(quote! {
            #kind => ::std::result::Result::Ok(#name::#op(#operation::new_args(&#op, before))),
        });
        apply_arms.push(quote! {
            #name::#op(args) => #operation::apply(&#op, part, operation_id, args, library),
        });
        picks_built_arms.push(quote! {
            #name::#op(_) => <#op as #operation>::PICKS_BUILT,
        });
        let session = quote!(<#op as #operation>::Session);
        let expect = quote!(.expect("a session made for a step of the same operation"));
        session_arms.push(quote! {
            #name::#op(_) => ::std::boxed::Box::new(<#session as ::std::default::Default>::default()),
        });
        form_arms.push(quote! {
            #name::#op(args) => #operation::form(
                &#op, context, args, session.downcast_ref::<#session>()#expect, selection,
            ).erase(),
        });
        set_arms.push(quote! {
            #name::#op(args) => #operation::set(
                &#op, context, args, session.downcast_mut::<#session>()#expect, selection,
                state, key, value,
            ),
        });
        event_arms.push(quote! {
            #name::#op(args) => #operation::event(
                &#op,
                context,
                ::geop_ops::ui::Edit {
                    args,
                    session: session.downcast_mut::<#session>()#expect,
                    selection,
                    state,
                },
                event,
            ),
        });
        formulas_arms.push(quote! {
            #name::#op(args) => #operation::formulas(&#op, args),
        });
        kind_arms.push(quote! { #name::#op(_) => #kind, });
        label_arms.push(quote! { #name::#op(_) => #label, });
        froms.push(quote! {
            impl ::std::convert::From<#args> for #name {
                fn from(args: #args) -> Self {
                    #name::#op(args)
                }
            }
        });
    }
    let private = quote!(::geop_ops::__private);
    quote! {
        impl ::geop_ops::operation::Operations for #name {
            fn infos() -> ::std::vec::Vec<::geop_ops::operation::OperationInfo> {
                ::std::vec![#(#infos),*]
            }

            fn new_step<S: #private::Scalar>(
                kind: &str,
                before: &::geop_ops::Part<S>,
            ) -> #private::GeopResult<Self> {
                match kind {
                    #(#new_arms)*
                    other => ::std::result::Result::Err(#private::GeopError::new(
                        ::std::format!("there is no operation {other:?}"),
                    )),
                }
            }

            fn apply<S: #private::Scalar>(
                &self,
                part: ::geop_ops::Part<S>,
                operation_id: &str,
                library: &dyn ::geop_ops::Library<S>,
            ) -> #private::GeopResult<::geop_ops::Part<S>> {
                match self {
                    #(#apply_arms)*
                }
            }

            fn picks_built(&self) -> bool {
                match self {
                    #(#picks_built_arms)*
                }
            }

            fn new_session(&self) -> ::std::boxed::Box<dyn ::std::any::Any> {
                match self {
                    #(#session_arms)*
                }
            }

            fn form<'a, S: #private::Scalar>(
                &self,
                context: ::geop_ops::Context<'a, S>,
                session: &dyn ::std::any::Any,
                selection: &[::std::string::String],
            ) -> ::geop_ops::ui::Form<'a, S> {
                match self {
                    #(#form_arms)*
                }
            }

            fn set<S: #private::Scalar>(
                &mut self,
                context: ::geop_ops::Context<'_, S>,
                session: &mut dyn ::std::any::Any,
                selection: &mut ::std::vec::Vec<::std::string::String>,
                state: &mut ::geop_ops::part::State,
                key: &str,
                value: ::geop_ops::ui::Value,
            ) {
                match self {
                    #(#set_arms)*
                }
            }

            fn event<S: #private::Scalar>(
                &mut self,
                context: ::geop_ops::Context<'_, S>,
                session: &mut dyn ::std::any::Any,
                selection: &mut ::std::vec::Vec<::std::string::String>,
                state: &mut ::geop_ops::part::State,
                event: &::geop_ops::ui::CanvasEvent<S>,
            ) {
                match self {
                    #(#event_arms)*
                }
            }

            fn formulas(&mut self) -> ::std::vec::Vec<&mut ::std::string::String> {
                match self {
                    #(#formulas_arms)*
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
        }
        #(#froms)*
    }
    .into()
}
