use std::usize;

use crate::larp::{Boundable, BoundingBox};

#[derive(Debug, Clone)]
pub enum BvhNode {
    Internal {
        bbox: BoundingBox,
        left: u32,
        right: u32,
    },

    Leaf {
        bbox: BoundingBox,
        start: u32,
        end: u32,
    },
}

impl BvhNode {
    pub fn new_internal(bbox: BoundingBox, left: usize, right: usize) -> Self {
        Self::Internal {
            bbox,
            left: left as u32,
            right: right as u32,
        }
    }

    pub fn new_leaf(bbox: BoundingBox, start: usize, end: usize) -> Self {
        Self::Leaf {
            bbox,
            start: start as u32,
            end: end as u32,
        }
    }
}

impl Boundable for BvhNode {
    fn bounding_box(&self) -> BoundingBox {
        match self {
            Self::Internal { bbox, .. } => bbox.clone(),
            Self::Leaf { bbox, .. } => bbox.clone(),
        }
    }
}

#[derive(Debug, Clone)]
pub enum LbvhNode {
    Internal {
        parent: Option<usize>,
        left: usize,
        right: usize,
        bbox: BoundingBox,
    },
    Leaf {
        parent: Option<usize>,
        primitive_index: usize,
        bbox: BoundingBox,
    },
}

impl LbvhNode {
    pub const EMPTY_LEAF: Self = Self::Leaf {
        parent: None,
        primitive_index: 0,
        bbox: BoundingBox::EMPTY,
    };

    #[inline(always)]
    #[must_use]
    pub const fn new_internal(
        parent: Option<usize>,
        bbox: BoundingBox,
        left: usize,
        right: usize,
    ) -> Self {
        Self::Internal {
            parent,
            bbox,
            left,
            right,
        }
    }

    #[inline(always)]
    #[must_use]
    pub const fn new_leaf(
        parent: Option<usize>,
        bbox: BoundingBox,
        primitive_index: usize,
    ) -> Self {
        Self::Leaf {
            parent,
            bbox,
            primitive_index,
        }
    }

    #[inline]
    #[must_use]
    pub const fn parent(&self) -> Option<usize> {
        match self {
            Self::Internal { parent, .. } | Self::Leaf { parent, .. } => *parent,
        }
    }

    #[inline]
    pub const fn set_parent(&mut self, parent: Option<usize>) {
        match self {
            Self::Internal { parent: p, .. } | Self::Leaf { parent: p, .. } => *p = parent,
        }
    }
}

impl Boundable for LbvhNode {
    fn bounding_box(&self) -> BoundingBox {
        match self {
            Self::Internal { bbox, .. } => bbox.clone(),
            Self::Leaf { bbox, .. } => bbox.clone(),
        }
    }
}
