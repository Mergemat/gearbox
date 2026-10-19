use std::path::PathBuf;
use gearbox_packs::{crystal, embedded, nodes, prefab::Ctx};

#[test]
fn deformation_items_match_the_original_compiler() {
    let Some(path) = std::env::var_os("GEARBOX_CRYSTAL_TEST_MAP").map(PathBuf::from) else {
        eprintln!("GEARBOX_CRYSTAL_TEST_MAP unset; skipping local map fixture");
        return;
    };
    let map = gearbox_gbx::Map::parse(&std::fs::read(path).unwrap()).unwrap();
    let cases = [
        (
            "DHA_G_Ci_3x3.Item.Gbx",
            1700,
            1088,
            0x7a3c934edb34d933u64,
            [
                0xc27ffd8a, 0x4200005d, 0xb6000000, 0x41900000, 0xc27fff29, 0x42000448,
            ],
        ),
        (
            "DHA_G_Co_3x3.Item.Gbx",
            1700,
            1088,
            0x643cd5e232fbfee4u64,
            [
                0xc280006c, 0x4200005d, 0xb6000000, 0x418fffff, 0x39f28000, 0x42c0000d,
            ],
        ),
        (
            "DHA_G_Co_4x4.Item.Gbx",
            2125,
            1360,
            0x45fbfea267d1c02fu64,
            [
                0xc2c0006d, 0x42000057, 0xb6400000, 0x418fffff, 0x39f28000, 0x43000005,
            ],
        ),
    ];
    for (name, vertices, faces, face_hash, bounds) in cases {
        let file = map
            .embedded
            .files
            .iter()
            .find(|f| f.path.ends_with(name))
            .unwrap();
        let buf = embedded::plain_gbx(&file.data).unwrap();
        let layout = nodes::layout(&buf).unwrap();
        let ctx = Ctx {
            buf: &buf,
            layout: &layout,
            path: &file.path,
        };
        let start = layout
            .nodes
            .iter()
            .find(|n| n.class_id == 0x09003000)
            .unwrap()
            .start;
        let c = crystal::read(&ctx, start, |_| Ok([0, 0])).unwrap();
        for g in [&c.geometry[0], &c.visual_geometry[0]] {
            assert_eq!(g.positions.len(), vertices, "{name}");
            assert_eq!(g.faces.len(), faces, "{name}");
            let actual: Vec<_> = (0..3)
                .flat_map(|axis| {
                    [
                        g.positions
                            .iter()
                            .map(|p| p[axis])
                            .fold(f32::INFINITY, f32::min)
                            .to_bits(),
                        g.positions
                            .iter()
                            .map(|p| p[axis])
                            .fold(f32::NEG_INFINITY, f32::max)
                            .to_bits(),
                    ]
                })
                .collect();
            assert_eq!(actual, bounds, "{name}");
            let mut bytes = Vec::new();
            for (face, material) in g.faces.iter().rev() {
                bytes.extend(material.to_le_bytes());
                bytes.extend((face.len() as u32).to_le_bytes());
                for &v in face {
                    bytes.extend(
                        g.positions[v as usize]
                            .into_iter()
                            .flat_map(f32::to_le_bytes),
                    );
                }
            }
            let hash = bytes.into_iter().fold(0xcbf29ce484222325u64, |h, b| {
                (h ^ b as u64).wrapping_mul(0x100000001b3)
            });
            assert_eq!(hash, face_hash, "{name}");
        }
    }
}
