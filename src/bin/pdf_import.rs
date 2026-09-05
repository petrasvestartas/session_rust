//! CLI shim over `session_rust::pdf::import_pdf`.
//!   pdf_import <file.pdf> <out_stem> [page]
fn main() {
    let mut args = std::env::args().skip(1);
    let src = args
        .next()
        .expect("usage: pdf_import <file.pdf> <out_stem> [page]");
    let stem = args
        .next()
        .unwrap_or_else(|| src.trim_end_matches(".pdf").to_string());
    let page_no = args.next().and_then(|p| p.parse::<i32>().ok()).unwrap_or(0);
    session_rust::pdf::import_pdf(&src, &stem, page_no);
}
