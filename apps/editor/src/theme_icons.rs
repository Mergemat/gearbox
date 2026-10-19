use std::borrow::Cow;

pub struct Icons;

impl gpui::AssetSource for Icons {
    fn load(&self, path: &str) -> anyhow::Result<Option<Cow<'static, [u8]>>> {
        let bytes: &'static [u8] = match path {
            "icons/projects.svg" => include_bytes!("../assets/icons/projects.svg"),
            "icons/map.svg" => include_bytes!("../assets/icons/map.svg"),
            "icons/download.svg" => include_bytes!("../assets/icons/download.svg"),
            "icons/flask.svg" => include_bytes!("../assets/icons/flask.svg"),
            "icons/folder.svg" => include_bytes!("../assets/icons/folder.svg"),
            "icons/more.svg" => include_bytes!("../assets/icons/more.svg"),
            "icons/box.svg" => include_bytes!("../assets/icons/box.svg"),
            "icons/brake.svg" => include_bytes!("../assets/icons/brake.svg"),
            "icons/branch.svg" => include_bytes!("../assets/icons/branch.svg"),
            "icons/brush.svg" => include_bytes!("../assets/icons/brush.svg"),
            "icons/car.svg" => include_bytes!("../assets/icons/car.svg"),
            "icons/check.svg" => include_bytes!("../assets/icons/check.svg"),
            "icons/clear.svg" => include_bytes!("../assets/icons/clear.svg"),
            "icons/eye.svg" => include_bytes!("../assets/icons/eye.svg"),
            "icons/eyeoff.svg" => include_bytes!("../assets/icons/eyeoff.svg"),
            "icons/finish.svg" => include_bytes!("../assets/icons/finish.svg"),
            "icons/fit.svg" => include_bytes!("../assets/icons/fit.svg"),
            "icons/gas.svg" => include_bytes!("../assets/icons/gas.svg"),
            "icons/gate.svg" => include_bytes!("../assets/icons/gate.svg"),
            "icons/gauge.svg" => include_bytes!("../assets/icons/gauge.svg"),
            "icons/ghost.svg" => include_bytes!("../assets/icons/ghost.svg"),
            "icons/goon.svg" => include_bytes!("../assets/icons/goon.svg"),
            "icons/height.svg" => include_bytes!("../assets/icons/height.svg"),
            "icons/help.svg" => include_bytes!("../assets/icons/help.svg"),
            "icons/history.svg" => include_bytes!("../assets/icons/history.svg"),
            "icons/keep.svg" => include_bytes!("../assets/icons/keep.svg"),
            "icons/line.svg" => include_bytes!("../assets/icons/line.svg"),
            "icons/list.svg" => include_bytes!("../assets/icons/list.svg"),
            "icons/lock.svg" => include_bytes!("../assets/icons/lock.svg"),
            "icons/move.svg" => include_bytes!("../assets/icons/move.svg"),
            "icons/pause.svg" => include_bytes!("../assets/icons/pause.svg"),
            "icons/play.svg" => include_bytes!("../assets/icons/play.svg"),
            "icons/plus.svg" => include_bytes!("../assets/icons/plus.svg"),
            "icons/point.svg" => include_bytes!("../assets/icons/point.svg"),
            "icons/pose.svg" => include_bytes!("../assets/icons/pose.svg"),
            "icons/protractor.svg" => include_bytes!("../assets/icons/protractor.svg"),
            "icons/range.svg" => include_bytes!("../assets/icons/range.svg"),
            "icons/record.svg" => include_bytes!("../assets/icons/record.svg"),
            "icons/rotate.svg" => include_bytes!("../assets/icons/rotate.svg"),
            "icons/route.svg" => include_bytes!("../assets/icons/route.svg"),
            "icons/ruler.svg" => include_bytes!("../assets/icons/ruler.svg"),
            "icons/scale.svg" => include_bytes!("../assets/icons/scale.svg"),
            "icons/select.svg" => include_bytes!("../assets/icons/select.svg"),
            "icons/setup.svg" => include_bytes!("../assets/icons/setup.svg"),
            "icons/sliders.svg" => include_bytes!("../assets/icons/sliders.svg"),
            "icons/sparkle.svg" => include_bytes!("../assets/icons/sparkle.svg"),
            "icons/steer.svg" => include_bytes!("../assets/icons/steer.svg"),
            "icons/stepback.svg" => include_bytes!("../assets/icons/stepback.svg"),
            "icons/stepfwd.svg" => include_bytes!("../assets/icons/stepfwd.svg"),
            "icons/stop.svg" => include_bytes!("../assets/icons/stop.svg"),
            "icons/stopwatch.svg" => include_bytes!("../assets/icons/stopwatch.svg"),
            "icons/target.svg" => include_bytes!("../assets/icons/target.svg"),
            "icons/tire.svg" => include_bytes!("../assets/icons/tire.svg"),
            "icons/tostart.svg" => include_bytes!("../assets/icons/tostart.svg"),
            "icons/unlock.svg" => include_bytes!("../assets/icons/unlock.svg"),
            _ => return Ok(None),
        };
        Ok(Some(Cow::Borrowed(bytes)))
    }

    fn list(&self, _path: &str) -> anyhow::Result<Vec<gpui::SharedString>> {
        Ok(Vec::new())
    }
}
