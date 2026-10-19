const HELP: &str = "usage: gearbox [--game DIR] <command> [args]

  setup [--game DIR]   find Trackmania, save it, build the pack cache (run this first)
  assets <command>     game packs: extract, glTF export, checks (`gearbox assets` lists them)
  search …             input search (`gearbox search --help`)
  sandbox …            placement, picking, reachability (`gearbox sandbox --help`)
  probe MAP INPUTS CSV trick signatures along a run
  golden <command>     golden traces and search regressions (`gearbox golden` lists them)

--game DIR uses the Trackmania folder (or its Packs folder) DIR for this run.

GBX files:";

fn main() -> anyhow::Result<()> {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    gearbox_cli::select_game(gearbox_cli::take_game(&mut args)?);
    let cmd = if args.is_empty() { String::from("help") } else { args.remove(0) };
    match cmd.as_str() {
        "setup" => gearbox_cli::setup::main(args),
        "assets" => gearbox_cli::assets::main(args),
        "search" => gearbox_cli::search::main(args),
        "golden" => gearbox_cli::golden::main(args),
        "sandbox" => gearbox_cli::sandbox::main(args),
        "probe" => gearbox_cli::probe::main(args),
        "help" | "-h" | "--help" => {
            println!("{HELP}\n{}", gearbox_cli::gbx::USAGE.trim_start_matches("usage:").trim_end());
            match gearbox_cli::game() {
                Ok(game) => println!("\nTrackmania packs: {}", game.packs.display()),
                Err(e) => println!("\n{e}"),
            }
            Ok(())
        }
        _ => gearbox_cli::gbx::main([cmd].into_iter().chain(args).collect()),
    }
}
