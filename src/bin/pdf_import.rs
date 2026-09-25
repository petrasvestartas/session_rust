use session_rust::pdf::import_pdf;

const ASSETS: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/../session_viewer/assets");
const PAGE: i32 = 0;
const SHEETS: [(&str, &str); 9] = [
    (
        "30700_querschnitt_gg",
        "pdf/Projekt-I/30700 Querschnitt G-G.pdf",
    ),
    (
        "draw_pb_haus25",
        "pdf/Projekt-B/Ansicht Haus 25 TH Schnitt a-a b-b.pdf",
    ),
    (
        "draw_pc_gru_og2",
        "pdf/Projekt-C/GRU_02 1149whz_ARC_AP_GRU_OG2_2.Obergeschoss.pdf",
    ),
    (
        "draw_pd_treppenhaus04",
        "pdf/Projekt-D/KAB_41_ARC_B2_MG_DE_5002_B2 Treppenhaus 04_F.pdf",
    ),
    (
        "draw_pe_schalungsbild",
        "pdf/Projekt-E/2508_W-700.1 Schalungsbild TRH West.pdf",
    ),
    ("draw_pf_he", "pdf/Projekt-F/HE.pdf"),
    (
        "draw_pi_laengsschnitt",
        "pdf/Projekt-I/30300 Längsschnitt C-C.pdf",
    ),
    (
        "draw_pj_grundriss_og2",
        "pdf/Projekt-J/1606.51.4054 Grundriss 2. Obergeschoss.pdf",
    ),
    (
        "draw_pj_treppenhaus_a",
        "pdf/Projekt-J/1606.41.3201 Treppenhaus A.pdf",
    ),
];

fn main() {
    std::fs::create_dir_all(format!("{ASSETS}/pb")).expect("cannot create pb directory");

    for (stem, pdf) in SHEETS {
        let src = format!("{ASSETS}/{pdf}");

        if std::path::Path::new(&src).exists() {
            import_pdf(&src, &format!("{ASSETS}/pb/{stem}"), PAGE);
        }
    }
}

/*
description: import every sheet in SHEETS from session_viewer/assets/pdf into assets/pb, fonts into assets/fonts.

directory: cd ~/code/code_cpp/wood_research/session/session_rust
run: cargo run --release --features pdf --bin pdf_import
*/
