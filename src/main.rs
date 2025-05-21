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
        let speed = 2.0;

        let t = optimal_t(model.tracktor_position, model.target_position, model.tracktor_angle, model.target_angle);

        let [p0, p1, z1, z0] = generate_base_points(model.tracktor_position, model.tracktor_angle, model.target_position, model.target_angle);

        let target = bezier_point(p0, p1, z1, z0, t);
        
        
        let angle = (target - model.tracktor_position).angle() - PI / 2.0;
        let mut angle = angle - model.tracktor_angle;

        while angle > PI {
            angle -= PI * 2.0;
        }
        while angle < -PI {
            angle += PI * 2.0;
        }

        let angle = angle.clamp(-1.0, 1.0);

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
    
    model.tracktor_angle += angle * update.since_last.as_secs_f32() * model.tracktor_speed * 1.0;

    // Update tracktor position based on speed and angle
    model.tracktor_position.x -= model.tracktor_speed * model.tracktor_angle.sin() * update.since_last.as_secs_f32() * 50.0;
    model.tracktor_position.y += model.tracktor_speed * model.tracktor_angle.cos() * update.since_last.as_secs_f32() * 50.0;


    if model.tracktor_position.distance(model.target_position) < 50.0 && normalize_angle(model.tracktor_angle - model.target_angle).abs() < 0.65 {
        model.new_target();
    }
}

fn generate_base_points(p0: Vec2, p0_dir: f32, p1: Vec2, p1_dir: f32) -> [Vec2; 4] {
    let dist = p0.distance(p1);
    let d = 200.0;
    let d2 = (dist / 2.0).max(400.0);
    let p3 = p0 + Vec2::new(0.0, d).rotate(p0_dir);
    let p4 = p1 + Vec2::new(0.0, d2).rotate(p1_dir + PI);
    
    if PI - normalize_angle(p1_dir - p0_dir).abs() < 1.2 {
        let dir_to_target = (p1 - p0).angle();
        let offset = if normalize_angle(p1_dir - p0_dir).abs() > PI {
            -PI / 4.0
        } else {
            PI / 4.0
        };
        let p0_dir = normalize_angle(dir_to_target + PI / 2.0);
        let p3 = p0 + Vec2::new(0.0, d).rotate(p0_dir - offset);
        let p4 = p1 + Vec2::new(0.0, d2*1.2).rotate(p1_dir + PI + offset);
        return [p0, p3, p4, p1];
    } 

    [p0, p3, p4, p1]
}

fn optimal_t(tractor_pos: Vec2, target_pos: Vec2, tracktor_angle: f32, target_angle: f32) -> f32 {
    let follow_point_dist = 250.0;
    let dist = tractor_pos.distance(target_pos);
    let t = follow_point_dist / dist;
    if dist <= 280.0 && PI - normalize_angle(target_angle - tracktor_angle).abs() < 1.3 {
        return 0.5;
    }
    t.clamp(0.1, 1.0)
}

fn bezier_point(p0: Vec2, p1: Vec2, p2: Vec2, p3: Vec2, t: f32) -> Vec2 {
    let u = 1.0 - t;
    let tt = t * t;
    let uu = u * u;
    let uuu = uu * u;
    let ttt = tt * t;

    uuu * p0 + 3.0 * uu * t * p1 + 3.0 * u * tt * p2 + ttt * p3
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
    let [p0, p1, z1, z0] = generate_base_points(model.tracktor_position, model.tracktor_angle, model.target_position, model.target_angle);

    draw.ellipse()
        .xy(p1)
        .radius(5.0)
        .color(YELLOW);

    draw.ellipse()
        .xy(z1)
        .radius(5.0)
        .color(YELLOW);

    draw.polyline()
        .points(vec![p0, p1, z1, z0])
        .color(WHITE);

    let curve = (0..=100)
        .map(|i| {
            let t = i as f32 / 100.0;
            bezier_point(p0, p1, z1, z0, t)            
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

    let t = optimal_t(model.tracktor_position, model.target_position, model.tracktor_angle, model.target_angle);

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

fn normalize_angle(angle: f32) -> f32 {
    let mut angle = angle;
    while angle > PI {
        angle -= PI * 2.0;
    }
    while angle < -PI {
        angle += PI * 2.0;
    }
    angle
}