use gpui::{AnyElement, Context, IntoElement, ParentElement, Styled, div, px};

use crate::app::*;
use crate::fmt::fmt_ms;
use crate::lab::RouteFrom;
use crate::ui::widgets::*;

impl Editor {
    pub fn route_props(&mut self, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let stops = self.route_stops();
        if !stops.iter().any(|s| s.at == self.route.to)
            && let Some(last) = stops.last()
        {
            self.route.to = last.at;
        }
        let ms0 = self.race_ms0;
        let at = move |t: Option<usize>| {
            t.map_or("not reached".to_string(), |k| format!("{} s", fmt_ms(ms0 + 10 * k as i32)))
        };
        let mut v: Vec<AnyElement> = vec![heading("Stretch").into_any_element()];
        let mut from: Vec<(String, RouteFrom)> = vec![("Race start".into(), RouteFrom::RaceStart)];
        if self.doc.start.is_some() {
            from.push(("The car".into(), RouteFrom::Placed));
        }
        for (i, s) in stops.iter().enumerate() {
            if s.tick.is_some() {
                from.push((format!("{}  {}", s.name, at(s.tick)), RouteFrom::Stop(i)));
            }
        }
        let sel = self.dropdown(
            "route.from",
            self.route.from,
            from,
            |e, v, _| {
                e.route.from = v;
                e.frame_route();
            },
            cx,
        );
        v.push(prop("From", sel).into_any_element());
        let to = stops.iter().map(|s| (format!("{}  {}", s.name, at(s.tick)), s.at)).collect();
        let sel = self.dropdown(
            "route.to",
            self.route.to,
            to,
            |e, v, _| {
                e.route.to = v;
                e.frame_route();
            },
            cx,
        );
        v.push(prop("To", sel).into_any_element());
        let first = match self.route.from {
            RouteFrom::Stop(i) => i + 1,
            _ => 0,
        };
        let through = match stops.iter().position(|s| s.at == self.route.to) {
            Some(l) if l >= first => {
                stops[first..=l].iter().map(|s| s.name.as_str()).collect::<Vec<_>>().join(" \u{2192} ")
            }
            _ => "To is before From".into(),
        };
        v.push(prop_top("Through", div().pt(px(5.0)).child(self.ro(through))).into_any_element());
        if let Some((a, b)) = self.route_ticks() {
            v.push(
                prop("Your run", self.ro(format!("{} s", fmt_ms(10 * (b - a) as i32)))).into_any_element(),
            );
        }
        v.push(heading("Search").into_any_element());
        let budget = self.budget_field("route-budget", self.route.seconds, |e, v, _| e.route.seconds = v, cx);
        v.push(prop("For", budget).into_any_element());
        v.push(
            prop("", self.run_button("route-find", "Find route", |e| e.find_route(), cx)).into_any_element(),
        );
        v.extend(self.takes(cx));
        v
    }
}

pub fn capital(s: &str) -> String {
    let mut c = s.chars();
    c.next().map_or(String::new(), |f| f.to_uppercase().collect::<String>() + c.as_str())
}

pub fn route_title(e: &Editor) -> String {
    format!("{} \u{2192} {}", e.route_from_name(), e.route_to_name())
}
