use nannou::{draw::{self, properties::spatial::position}, prelude::*};

fn main() {
    nannou::app(model).update(update).run();
}

const TRACKTOR_WIDTH: f32 = 20.0;
const TRACKTOR_LENGTH: f32 = 35.0;

struct Model {
    tracktor_position: Vec2,
    tracktor_angle: f32,
    tracktor_speed: f32,
    target_position: Vec2,
    target_angle: f32,
    obstacles: Vec<Obstacle>,
    drive_to_point_behavior: DriveToPointBehavior,
}

impl Model {
    fn new_target(&mut self) {
        let angle = random_f32() * 2.0 * PI;
        let position = Vec2::new(
            random_range(-500.0, 500.0),
            random_range(-500.0, 500.0),
        );

        self.target_position = position;
        self.target_angle = angle;
        self.drive_to_point_behavior.target_position = position;
        self.drive_to_point_behavior.target_angle = angle;
    }
}


struct DriveToPointBehavior {
    target_position: Vec2,
    target_angle: f32,

    // Meta signal input
    stimulation: f32,
    inhibitance: f32,
}

impl DriveToPointBehavior {
    fn new() -> Self {
        Self {
            target_position: Vec2::ZERO,
            target_angle: 0.0,
            stimulation: 0.0,
            inhibitance: 0.0,
        }
    }


    fn output(&self, model: &Model) -> (f32, f32) {
        let speed = 1.0;

        let dist = model.tracktor_position.distance(model.target_position);

        let d = dist / 2.0;
        let d = d.max(400.0);

        let p0 = model.tracktor_position;

        let z0 = model.target_position;
        let z1 = model.target_position + Vec2::new(0.0, d).rotate(model.target_angle + PI);

        // let t = match dist {
        //     ..200.0 => 1.0,
        //     200.0..300.0 => 0.9,
        //     300.0..=400.0 => 0.7,
        //     400.0..=500.0 => 0.5,
        //     300.0.. => 0.3,
        //     _ => 0.0,
        // };

        let follow_point_dist = 150.0;
        let t = follow_point_dist / dist;
        
        let t = t.clamp(0.1, 1.0);

        let target = (1.0 - t).powi(2) * p0 + 2.0 * (1.0 - t) * t * z1 + t.powi(2) * z0;
        
        
        let angle = (target - model.tracktor_position).angle() - PI / 2.0;
        let mut angle = angle - model.tracktor_angle;

        while angle > PI {
            angle -= PI * 2.0;
        }
        while angle < -PI {
            angle += PI * 2.0;
        }

        (speed, angle)
    }
}

struct Obstacle {
    position: Vec2,
    radius: f32,
}


fn model(app: &App) -> Model {
    // Create a window that can receive user input like mouse and keyboard events.
    app.new_window()
        .size(1200, 1200)
        .view(view)
        .build()
        .unwrap();
    
    let mut model = Model {
        tracktor_position: Vec2::new(-300.0, -300.0),
        tracktor_angle: PI,
        tracktor_speed: 0.0,
        target_position: Vec2::new(400.0, 400.0),
        target_angle: PI,
        obstacles: vec![
            Obstacle {
                position: Vec2::new(200.0, 200.0),
                radius: 100.0,
            },
            Obstacle {
                position: Vec2::new(-200.0, 200.0),
                radius: 100.0,
            },
        ],
        drive_to_point_behavior: DriveToPointBehavior::new(),
    };

    model.drive_to_point_behavior.target_position = model.target_position;
    model.drive_to_point_behavior.target_angle = model.target_angle;

    model
}


fn update(_app: &App, model: &mut Model, update: Update) {
    let (speed, angle) = model.drive_to_point_behavior.output(model);
    let speed_diff = (speed - model.tracktor_speed).clamp(-0.5, 0.5);
    model.tracktor_speed += speed_diff * update.since_last.as_secs_f32();

    let angle = angle.clamp(-0.5, 0.5);
    
    model.tracktor_angle += angle * update.since_last.as_secs_f32() * model.tracktor_speed;

    // Update tracktor position based on speed and angle
    model.tracktor_position.x -= model.tracktor_speed * model.tracktor_angle.sin() * update.since_last.as_secs_f32() * 50.0;
    model.tracktor_position.y += model.tracktor_speed * model.tracktor_angle.cos() * update.since_last.as_secs_f32() * 50.0;


    if model.tracktor_position.distance(model.target_position) < 50.0 {
        model.new_target();
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = draw::Draw::new();
    draw.background().color(BLACK);

    // for obstacle in &model.obstacles {
    //     draw.ellipse()
    //         .xy(obstacle.position)
    //         .radius(obstacle.radius)
    //         .color(PURPLE);
    // }
    draw.rect()
        .xy(model.tracktor_position)
        .w_h(TRACKTOR_WIDTH, TRACKTOR_LENGTH)
        .rotate(model.tracktor_angle)
        .color(BLUE);

    draw.ellipse()
        .xy(model.target_position)
        .radius(10.0)
        .color(RED);

    draw.arrow()
        .start(model.target_position)
        .end(model.target_position + Vec2::new(0.0, 50.0).rotate(model.target_angle))
        .color(RED)
        .weight(5.0);

    draw.arrow()
        .start(model.tracktor_position)
        .end(model.tracktor_position + Vec2::new(0.0, 50.0).rotate(model.tracktor_angle))
        .color(BLUE)
        .weight(5.0);

    // draw Bezier curve
    let dist = model.tracktor_position.distance(model.target_position);
    let d = dist / 2.0;
    let d = d.max(400.0);

    let p0 = model.tracktor_position;
    
    let z0 = model.target_position;
    let z1 = model.target_position + Vec2::new(0.0,  d).rotate(model.target_angle + PI);


    draw.ellipse()
        .xy(z1)
        .radius(5.0)
        .color(YELLOW);

    draw.polyline()
        .points(vec![p0, z1, z0])
        .color(WHITE);

    let curve = (0..=100)
        .map(|i| {
            let t = i as f32 / 100.0;
            (1.0 - t).powi(2) * p0 + 2.0 * (1.0 - t) * t * z1 + t.powi(2) * z0
        })
        .collect::<Vec<_>>();
    
    //     let t = match dist {
    //         ..200.0 => 0.99,
    //         200.0..300.0 => 0.9,
    //         300.0..=400.0 => 0.7,
    //         400.0..=500.0 => 0.5,
    //         300.0.. => 0.3,
    //         _ => 0.0,
    //     };

    let follow_point_dist = 150.0;
    let t = follow_point_dist / dist;
    let t = t.clamp(0.1, 1.0);
    let indx = (t * 100.0) as usize;

    draw.ellipse()
        .xy(curve[indx])
        .radius(5.0)
        .color(RED);
    
    draw.polyline()
        .points(curve)
        .color(GREEN);

    
    let _ = draw.to_frame(app, &frame);
}