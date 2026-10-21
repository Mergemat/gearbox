use std::collections::HashMap;

use anyhow::{Result, ensure};

use crate::unpack::{GbxHead, is_class, parse_head};

#[derive(Clone, Debug)]
pub struct NodeSpan {
    pub index: u32,
    pub class_id: u32,
    pub start: usize,
    pub end: Option<usize>,
}

pub struct Layout {
    pub head: GbxHead,
    pub nodes: Vec<NodeSpan>,
    pub by_start: HashMap<usize, usize>,
}

const FACADE: [u8; 4] = 0xfacade01u32.to_le_bytes();
const SPRITE_PARAMS: [u8; 4] = 0x0901_0008u32.to_le_bytes();

pub fn layout(buf: &[u8]) -> Result<Layout> {
    let head = match parse_head(buf) {
        Some(h) => h?,
        None => anyhow::bail!("truncated GBX header"),
    };
    ensure!(!head.body_compressed, "LZO-compressed body");
    let mut nodes = Vec::new();
    let mut next = 0u32;
    let advance = |n: &mut u32| {
        *n += 1;
        while head.refs.contains(n) {
            *n += 1;
        }
    };
    advance(&mut next);
    let main_chunked = buf.len() >= head.body_start + 4
        && is_class(
            u32::from_le_bytes(buf[head.body_start..head.body_start + 4].try_into().unwrap()) & !0xfff,
        );
    let mut stack: Vec<usize> = Vec::new();
    nodes.push(NodeSpan { index: 0, class_id: head.class_id, start: head.body_start, end: None });
    if main_chunked {
        stack.push(0);
    }
    let mut p = head.body_start;
    while p + 4 <= buf.len() {
        if buf.get(p..p + 8) == Some(&[4, 0x30, 0, 9, b'P', b'I', b'K', b'S']) {
            let size = buf.get(p + 8..p + 12).map(|b| u32::from_le_bytes(b.try_into().unwrap()) as usize);
            let end = size.and_then(|n| (p + 12).checked_add(n)).filter(|&e| e <= buf.len());
            ensure!(end.is_some(), "truncated crystal editor cache at {p}");
            p = end.unwrap();
            continue;
        }
        if buf[p..p + 4] == FACADE && !stack.is_empty() {
            let i = stack.pop().unwrap();
            nodes[i].end = Some(p + 4);
            p += 4;
            continue;
        }
        if buf.get(p..p + 8) == Some(&[1, 0xa0, 6, 9, 1, 0, 0, 0]) {
            let chunk = buf.get(p + 8..p + 12).map(|b| u32::from_le_bytes(b.try_into().unwrap()));
            ensure!(matches!(chunk, Some(0x0905_7000 | 0x0905_7001)), "invalid direct index buffer at {p}");
            nodes.push(NodeSpan { index: u32::MAX, class_id: 0x09057000, start: p + 8, end: None });
            stack.push(nodes.len() - 1);
            p += 8;
            continue;
        }
        if buf[p..p + 4] == SPRITE_PARAMS
            && let Some(chunk) = buf.get(p + 4..p + 8).map(|b| u32::from_le_bytes(b.try_into().unwrap()))
            && is_class(chunk & !0xfff)
        {
            nodes.push(NodeSpan { index: u32::MAX, class_id: chunk & !0xfff, start: p + 4, end: None });
            stack.push(nodes.len() - 1);
            p += 4;
            continue;
        }
        if p + 8 <= buf.len() {
            let w = u32::from_le_bytes(buf[p..p + 4].try_into().unwrap());
            if w == next && w < head.num_nodes {
                let cid = u32::from_le_bytes(buf[p + 4..p + 8].try_into().unwrap());
                if is_class(cid) {
                    let chunked = buf
                        .get(p + 8..p + 12)
                        .is_some_and(|b| is_class(u32::from_le_bytes(b.try_into().unwrap()) & !0xfff));
                    nodes.push(NodeSpan { index: w, class_id: cid, start: p + 8, end: None });
                    if chunked {
                        stack.push(nodes.len() - 1);
                    }
                    advance(&mut next);
                    p += 8;
                    continue;
                }
            }
        }
        p += 1;
    }
    let by_start = nodes.iter().enumerate().map(|(i, n)| (n.start, i)).collect();
    Ok(Layout { head, nodes, by_start })
}
