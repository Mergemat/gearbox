use std::process::Command;

#[test]
fn golden_help_returns_before_running_any_subcommand() {
    for command in [
        &[][..], &["check"][..], &["record"][..], &["list"][..], &["show"][..],
        &["search"][..], &["search", "check"][..], &["search", "record"][..],
    ] {
        for help in ["--help", "-h"] {
            let output = Command::new(env!("CARGO_BIN_EXE_gearbox"))
                .arg("golden").args(command).arg(help)
                .output().unwrap();
            assert!(output.status.success(), "{command:?} {help}: {}",
                String::from_utf8_lossy(&output.stderr));
            let stdout = String::from_utf8(output.stdout).unwrap();
            assert!(stdout.contains("check"), "{stdout}");
            assert!(stdout.starts_with("golden <command>\n"), "{stdout}");
        }
    }
}
