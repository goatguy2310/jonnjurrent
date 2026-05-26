use renderer::TriangleMeshBuilder;

pub fn cat() -> TriangleMeshBuilder {
    TriangleMeshBuilder::new()
        .soup_from_obj("assets/cat/cat.obj")
        .scale_translate(0.6, [0., -10., 0.])
}

pub fn lucky() -> TriangleMeshBuilder {
    TriangleMeshBuilder::new()
        .soup_from_obj("assets/lucky.obj")
        .scale_translate(0.1, [0., 20., 0.])
}

pub fn maria() -> TriangleMeshBuilder {
    TriangleMeshBuilder::new()
        .soup_from_obj("assets/maria/Maria_C.obj")
        .scale_translate(15., [0., -25., 0.])
}
