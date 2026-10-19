use gearbox_search::aim::{Aim, Condition, Judged, Measure, Quantity, Wheels};
use gearbox_search::tricks::TrickKind;
use gpui::prelude::*;
use gpui::{AnyElement, Context, div, px};

use crate::app::*;
use crate::optimize::quantity_label;
use crate::theme::{self, c};
use crate::ui::widgets::*;

#[derive(Clone, Copy, Debug, PartialEq)]
enum MeasureType {
    Speed,
    Height,
    Distance,
    Angle,
    Yaw,
    Pitch,
    Roll,
    Trick,
}

impl MeasureType {
    fn of(q: &Quantity) -> Self {
        match q {
            Quantity::Speed => Self::Speed,
            Quantity::Height => Self::Height,
            Quantity::Distance { .. } => Self::Distance,
            Quantity::Angle { .. } => Self::Angle,
            Quantity::Yaw => Self::Yaw,
            Quantity::Pitch => Self::Pitch,
            Quantity::Roll => Self::Roll,
            Quantity::Trick(_) => Self::Trick,
        }
    }

    fn fresh(self, point: [f32; 3]) -> Quantity {
        match self {
            Self::Speed => Quantity::Speed,
            Self::Height => Quantity::Height,
            Self::Distance => Quantity::Distance { to: point },
            Self::Angle => Quantity::Angle { yaw: None, pitch: Some(0.0), roll: None },
            Self::Yaw => Quantity::Yaw,
            Self::Pitch => Quantity::Pitch,
            Self::Roll => Quantity::Roll,
            Self::Trick => Quantity::Trick(TrickKind::Nosebug),
        }
    }
}

fn item(k: MeasureType) -> (String, MeasureType) {
    let (name, _, icon) = quantity_label(&k.fresh([0.0; 3]));
    (format!("@{icon}|{name}"), k)
}

fn presets() -> Vec<(&'static str, Aim)> {
    vec![
        ("Fastest", Aim::fastest()),
        ("Longest nosebug", Aim::longest_trick(TrickKind::Nosebug)),
        ("Longest bugslide", Aim::longest_trick(TrickKind::Bugslide)),
        ("Most uberbug", Aim::longest_trick(TrickKind::Uberbug)),
        ("Nose position", Aim::nose_position()),
        ("Land flat", Aim::land_flat()),
    ]
}

impl Editor {
    fn aim_point(&self) -> [f32; 3] {
        self.doc.goals.first().map_or([0.0; 3], |g| g.pos().to_array())
    }

    pub fn aim_rows(&mut self, cx: &mut Context<Self>) -> Vec<AnyElement> {
        let aim = self.spec.aim.clone();
        let mut v: Vec<AnyElement> = Vec::new();
        let named = presets();
        let current = named.iter().position(|p| p.1 == aim);
        let mut items: Vec<(String, Option<usize>)> =
            named.iter().enumerate().map(|(i, p)| (p.0.to_string(), Some(i))).collect();
        if current.is_none() {
            items.insert(0, ("Custom".into(), None));
        }
        let sel = self.dropdown(
            "aim.preset",
            current,
            items,
            |e, i, _| {
                if let Some(i) = i {
                    e.spec.aim = presets().swap_remove(i).1;
                }
            },
            cx,
        );
        v.push(prop("Preset", sel).into_any_element());
        let when = self.choice(
            "aim.when",
            aim.judged,
            vec![("Moment", Judged::End), ("Best in range", Judged::Best)],
            |e, j, _| e.spec.aim.judged = j,
            cx,
        );
        v.push(prop("When", when).id("aim-when").tooltip(tip("When it is judged", Some("Moment: at the time below. Best in range: at the tick where it is best, up to that time."))).into_any_element());
        let point = self.aim_point();
        let kinds = [
            MeasureType::Speed,
            MeasureType::Angle,
            MeasureType::Height,
            MeasureType::Distance,
            MeasureType::Pitch,
            MeasureType::Yaw,
            MeasureType::Roll,
            MeasureType::Trick,
        ];
        let sel = self.dropdown(
            "aim.measure",
            MeasureType::of(&aim.measure.quantity),
            kinds.iter().map(|k| item(*k)).collect(),
            move |e, k, _| {
                if MeasureType::of(&e.spec.aim.measure.quantity) != k {
                    let quantity = k.fresh(point);
                    let higher = match k {
                        MeasureType::Angle | MeasureType::Distance => false,
                        MeasureType::Trick => true,
                        _ => e.spec.aim.measure.higher,
                    };
                    e.spec.aim.measure = Measure { quantity, higher };
                }
            },
            cx,
        );
        let way = match MeasureType::of(&aim.measure.quantity) {
            MeasureType::Angle | MeasureType::Distance => div()
                .text_size(px(theme::FS_SMALL))
                .text_color(c(theme::MUTED))
                .child("closer")
                .into_any_element(),
            MeasureType::Trick => div()
                .text_size(px(theme::FS_SMALL))
                .text_color(c(theme::MUTED))
                .child("longer")
                .into_any_element(),
            _ => self.choice(
                "aim.higher",
                aim.measure.higher,
                vec![("Higher", true), ("Lower", false)],
                |e, h, _| e.spec.aim.measure.higher = h,
                cx,
            ),
        };
        v.push(
            prop("Measure", div().flex().items_center().gap(px(theme::SP2)).w_full().child(sel).child(way))
                .into_any_element(),
        );
        match aim.measure.quantity {
            Quantity::Angle { yaw, pitch, roll } => {
                v.push(prop("", self.axes(yaw, pitch, roll, cx)).into_any_element())
            }
            Quantity::Distance { to } => {
                let goals: Vec<(String, [i32; 3])> =
                    self.doc.goals.iter().map(|g| (g.name.clone(), key(g.pos().to_array()))).collect();
                let points: Vec<[f32; 3]> = self.doc.goals.iter().map(|g| g.pos().to_array()).collect();
                let sel = self.dropdown(
                    "aim.to",
                    key(to),
                    goals,
                    move |e, k, _| {
                        if let Some(p) = points.iter().find(|p| key(**p) == k) {
                            e.spec.aim.measure.quantity = Quantity::Distance { to: *p };
                        }
                    },
                    cx,
                );
                v.push(prop("To", sel).into_any_element());
            }
            Quantity::Trick(kind) => {
                use TrickKind::*;
                let sel = self.choice(
                    "aim.trick",
                    kind,
                    vec![("Nosebug", Nosebug), ("Bugslide", Bugslide), ("Uberbug", Uberbug), ("Any", Any)],
                    |e, k, _| e.spec.aim.measure.quantity = Quantity::Trick(k),
                    cx,
                );
                v.push(prop("Trick", sel).into_any_element());
            }
            _ => {}
        }
        v.push(
            div()
                .flex()
                .justify_between()
                .items_baseline()
                .child(heading("Must hold"))
                .child(
                    div()
                        .px(px(theme::PAD))
                        .text_size(px(theme::FS_SMALL))
                        .text_color(c(theme::FAINT))
                        .child("else it ranks below"),
                )
                .into_any_element(),
        );
        for (i, cond) in aim.conditions.iter().enumerate() {
            v.push(self.condition_row(i, *cond, cx));
        }
        let add = self.dropdown(
            "aim.add",
            0usize,
            vec![
                ("+ Condition".into(), 0),
                ("@icons/gauge.svg|Speed over".into(), 1),
                ("@icons/height.svg|Height over".into(), 2),
                ("@icons/protractor.svg|Pitch under".into(), 3),
                ("@icons/tire.svg|Wheels touching".into(), 4),
            ],
            |e, k, _| {
                let c = match k {
                    1 => Condition::Above(Quantity::Speed, 300.0),
                    2 => Condition::Above(Quantity::Height, 0.0),
                    3 => Condition::Below(Quantity::Pitch, -60.0),
                    4 => Condition::Wheels { which: Wheels::Front, touching: true },
                    _ => return,
                };
                e.spec.aim.conditions.push(c);
            },
            cx,
        );
        v.push(div().px(px(theme::PAD)).w(px(180.0)).child(add).into_any_element());
        v.push(heading("Then").into_any_element());
        let ties = [
            None,
            Some(MeasureType::Speed),
            Some(MeasureType::Height),
            Some(MeasureType::Pitch),
            Some(MeasureType::Yaw),
            Some(MeasureType::Roll),
        ];
        let items: Vec<(String, Option<MeasureType>)> =
            ties.iter().map(|t| t.map_or(("Nothing".into(), None), |k| (item(k).0, Some(k)))).collect();
        let sel = self.dropdown(
            "aim.then",
            aim.then.map(|m| MeasureType::of(&m.quantity)),
            items,
            move |e, k, _| {
                e.spec.aim.then = k.map(|k| Measure {
                    quantity: k.fresh(point),
                    higher: e.spec.aim.then.is_none_or(|m| m.higher),
                });
            },
            cx,
        );
        let mut row = div().flex().items_center().gap(px(theme::SP2)).w_full().child(sel);
        if let Some(m) = aim.then {
            row = row.child(self.choice(
                "aim.then.higher",
                m.higher,
                vec![("Higher", true), ("Lower", false)],
                |e, h, _| {
                    if let Some(m) = &mut e.spec.aim.then {
                        m.higher = h;
                    }
                },
                cx,
            ));
        }
        v.push(prop("Tie-break", row).into_any_element());
        v
    }

    fn axes(
        &mut self,
        yaw: Option<f32>,
        pitch: Option<f32>,
        roll: Option<f32>,
        cx: &mut Context<Self>,
    ) -> AnyElement {
        let mut row = div().flex().items_center().gap(px(theme::SP2)).w_full();
        for (axis, value) in [(0usize, yaw), (1, pitch), (2, roll)] {
            let name = ["yaw", "pitch", "roll"][axis];
            let set = move |e: &mut Editor, v: Option<f32>| {
                if let Quantity::Angle { yaw, pitch, roll } = &mut e.spec.aim.measure.quantity {
                    *[yaw, pitch, roll][axis] = v;
                }
            };
            row = row.child(checkbox(format!("aim.axis.{axis}"), value.is_some(), name).on_click(
                cx.listener(move |this, _, _, cx| {
                    set(this, if value.is_some() { None } else { Some(0.0) });
                    cx.notify();
                }),
            ));
            if let Some(a) = value {
                row = row.child(div().w(px(58.0)).child(self.num(
                    format!("aim.axis.{axis}.v"),
                    f64::from(a),
                    Num { decimals: 0, ..DEG }.range(-180.0, 180.0),
                    move |e, x, _| set(e, Some(x as f32)),
                    cx,
                )));
            }
        }
        row.into_any_element()
    }

    fn condition_row(&mut self, i: usize, cond: Condition, cx: &mut Context<Self>) -> AnyElement {
        let mut row = div()
            .flex()
            .items_center()
            .gap(px(theme::SP2))
            .w_full()
            .px(px(theme::PAD))
            .min_h(px(theme::H_ROW));
        let edit = move |e: &mut Editor, f: &dyn Fn(&mut Condition)| {
            if let Some(c) = e.spec.aim.conditions.get_mut(i) {
                f(c);
            }
        };
        match cond {
            Condition::Above(q, x) | Condition::Below(q, x) => {
                let kinds = [
                    MeasureType::Speed,
                    MeasureType::Height,
                    MeasureType::Pitch,
                    MeasureType::Yaw,
                    MeasureType::Roll,
                ];
                let sel = self.dropdown(
                    &format!("aim.c{i}.q"),
                    MeasureType::of(&q),
                    kinds.iter().map(|k| item(*k)).collect(),
                    move |e, k, _| {
                        edit(e, &|c: &mut Condition| match c {
                            Condition::Above(q, _) | Condition::Below(q, _) => *q = k.fresh([0.0; 3]),
                            _ => {}
                        })
                    },
                    cx,
                );
                let above = matches!(cond, Condition::Above(..));
                let cmp = self.choice(
                    &format!("aim.c{i}.cmp"),
                    above,
                    vec![(">", true), ("<", false)],
                    move |e, a, _| {
                        edit(e, &|c: &mut Condition| {
                            if let Condition::Above(q, v) | Condition::Below(q, v) = *c {
                                *c = if a { Condition::Above(q, v) } else { Condition::Below(q, v) };
                            }
                        })
                    },
                    cx,
                );
                let (_, unit, _) = quantity_label(&q);
                let f = self.num(
                    format!("aim.c{i}.v"),
                    f64::from(x),
                    Num {
                        decimals: 0,
                        unit,
                        step: 1.0,
                        min: f64::NEG_INFINITY,
                        max: f64::INFINITY,
                        width: 0.0,
                    },
                    move |e, v, _| {
                        edit(e, &|c: &mut Condition| {
                            if let Condition::Above(_, x) | Condition::Below(_, x) = c {
                                *x = v as f32;
                            }
                        })
                    },
                    cx,
                );
                row = row.child(sel).child(cmp).child(div().w(px(96.0)).flex_none().child(f));
            }
            Condition::Wheels { which, touching } => {
                let which = self.choice(
                    &format!("aim.c{i}.w"),
                    which,
                    vec![
                        ("Front", Wheels::Front),
                        ("Rear", Wheels::Rear),
                        ("All", Wheels::All),
                        ("Any", Wheels::Any),
                    ],
                    move |e, w, _| {
                        edit(e, &|c: &mut Condition| {
                            if let Condition::Wheels { which, .. } = c {
                                *which = w;
                            }
                        })
                    },
                    cx,
                );
                let on = self.choice(
                    &format!("aim.c{i}.t"),
                    touching,
                    vec![("touching", true), ("off", false)],
                    move |e, t, _| {
                        edit(e, &|c: &mut Condition| {
                            if let Condition::Wheels { touching, .. } = c {
                                *touching = t;
                            }
                        })
                    },
                    cx,
                );
                row = row.child(icon("icons/tire.svg", theme::MUTED, 14.0)).child(which).child(on);
            }
            Condition::Inside { .. } => {
                row = row.child(div().flex_1().text_size(px(theme::FS_BODY)).child("Inside the box"))
            }
        }
        row.child(div().flex_1())
            .child(icon_btn(format!("aim.c{i}.x"), "\u{00d7}", "Remove").on_click(cx.listener(
                move |this, _, _, cx| {
                    if i < this.spec.aim.conditions.len() {
                        this.spec.aim.conditions.remove(i);
                    }
                    cx.notify();
                },
            )))
            .into_any_element()
    }
}

fn key(p: [f32; 3]) -> [i32; 3] {
    p.map(|v| (v * 100.0).round() as i32)
}
