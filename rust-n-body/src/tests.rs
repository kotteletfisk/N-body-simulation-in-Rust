#[cfg(test)]
mod tests {

    use crate::*;
    use test::Bencher;

    #[test]
    fn test_hue_conversion_1() {
        assert_eq!(mass_to_hue(1.0, 1.0, 1.0), 1.0);
    }
    #[test]
    fn test_hue_conversion_5000() {
        assert_eq!(mass_to_hue(5000.0, 5000.0, 5000.0), 1.0);
    }
    #[test]
    fn test_hue_conversion_10() {
        assert_eq!(mass_to_hue(2500.0, 0.0, 5000.0), 0.5);
    }

    #[bench]
    fn test_frame_benchmarks(bencher: &mut Bencher) {
        let mut app = App::new();

        app.add_plugins(MinimalPlugins);
        app.insert_resource(SimulationSettings::default());
        app.add_message::<ResetMessage>();

        app.add_systems(Startup, add_bodies);
        app.add_systems(
            Update,
            (build_quadtree, compute_physics, update_positions).chain(),
        );

        app.update(); // run startup

        bencher.iter(|| {
            std::hint::black_box( app.update());
        });
    }

    // bencher.iter(|| {
    //     for _ in 0..100 {
    //         app.update();
    //     }
    // });
}
