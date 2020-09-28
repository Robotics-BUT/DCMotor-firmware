// use proc_macro::TokenStream;
//
// trait PrintOnError {
//     fn print_on_err(&self, message: TokenStream);
// }
//
// impl<T, E> PrintOnError for Result<T, E> {
//     fn print_on_err(&self, message: TokenStream) {
//         match self {
//             Ok(_) => {}
//             Err(_) => defmt::error!(message),
//         }
//     }
// }
