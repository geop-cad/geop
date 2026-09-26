macro_rules! define_ids {
    ($($name:ident),* $(,)?) => {
        $(
            #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
            pub struct $name(pub u64);

            impl std::fmt::Display for $name {
                fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
                    write!(f, "{}({})", stringify!($name), self.0)
                }
            }

            impl From<$name> for u64 {
                fn from(id: $name) -> u64 {
                    id.0
                }
            }
        )*
    };
}

define_ids!(VertexId, EdgeId, CoedgeId, FaceId, ShellId, SolidId,);
