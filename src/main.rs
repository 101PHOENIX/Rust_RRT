use macroquad::prelude::*;
use ::rand::Rng;
use ::rand::rngs::ThreadRng;

// İki boyutlu bir noktayı temsil eden yapı
#[derive(Clone, Copy)]
struct Point {
    x: f32,
    y: f32,
}

impl Point {
    // İki nokta arasındaki öklid mesafesini hesaplayan fonksiyon
    fn distance(&self, other: &Point) -> f32 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }
}

// Düğüm yapısı, bir nokta ve ebeveyn indeksini içerir
struct Node {
    point: Point,
    parent: Option<usize>,
}

impl Node {
    // Yeni bir düğüm oluşturur, noktayı ve ebeveynini alır
    fn new(point: Point, parent: Option<usize>) -> Self {
        Node { point, parent }
    }
}

// RRT ağacını tanımlayan yapı
struct RRT {
    nodes: Vec<Node>, // Ağacın düğümleri
    goal: Point, // Hedef nokta
    step_size: f32, // Adım boyutu
    goal_threshold: f32, // Hedef eşiği
    rng: ThreadRng, // Rastgele sayı üreteci
}

impl RRT {
    // Başlangıç ve hedef noktalar, adım boyutu ve hedef eşiği ile yeni bir RRT ağacı oluşturur
    fn new(start: Point, goal: Point, step_size: f32, goal_threshold: f32) -> Self {
        let root = Node::new(start, None); // Başlangıç düğümünü kök olarak ekler
        RRT {
            nodes: vec![root],
            goal,
            step_size,
            goal_threshold,
            rng: ::rand::thread_rng(),
        }
    }

    // Rastgele bir nokta seçer
    fn random_point(&mut self, min_x: f32, max_x: f32, min_y: f32, max_y: f32) -> Point {
        let x = self.rng.gen_range(min_x..max_x);
        let y = self.rng.gen_range(min_y..max_y);
        Point { x, y }
    }

    // En yakın düğümü bulur
    fn find_nearest(&self, point: &Point) -> usize {
        self.nodes
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                a.point
                    .distance(point)
                    .partial_cmp(&b.point.distance(point))
                    .unwrap()
            })
            .map(|(index, _)| index)
            .unwrap()
    }

    // Verilen noktadan hedefe doğru adım boyutunda ilerler
    fn steer(&self, from: &Point, to: &Point) -> Point {
        let angle = (to.y - from.y).atan2(to.x - from.x);
        Point {
            x: from.x + self.step_size * angle.cos(),
            y: from.y + self.step_size * angle.sin(),
        }
    }

    // Çarpışma kontrolü, burada basit olarak her zaman çarpışmasız kabul edilir
    fn is_collision_free(&self, _point: &Point) -> bool {
        true
    }

    // Yeni düğüm ekler
    fn add_node(&mut self, point: Point, parent_index: usize) {
        let new_node = Node::new(point, Some(parent_index));
        self.nodes.push(new_node);
    }

    // Geriye doğru giderek yolu çıkarır
    fn trace_path(&self) -> Vec<Point> {
        let mut path = Vec::new();
        let mut current_node_index = self.nodes.len() - 1;

        while let Some(parent_index) = self.nodes[current_node_index].parent {
            path.push(self.nodes[current_node_index].point);
            current_node_index = parent_index;
        }
        path.push(self.nodes[current_node_index].point);
        path.reverse();
        path
    }
}

#[macroquad::main("RRT Visualization")]
async fn main() {
    let mut rng = ::rand::thread_rng();
    
    let start = Point {
        x: rng.gen_range(0.0..400.0),
        y: rng.gen_range(0.0..400.0),
    };
    let goal = Point {
        x: rng.gen_range(0.0..400.0),
        y: rng.gen_range(0.0..400.0),
    };
    
    let mut rrt = RRT::new(start, goal, 10.0, 10.0);
    let mut goal_reached = false;
    let mut optimal_path: Vec<Point> = Vec::new();

    loop {
        if !goal_reached {
            // Rastgele nokta oluştur ve ağaçta en yakın düğümü bul
            let rand_point = rrt.random_point(0.0, 400.0, 0.0, 400.0);
            let nearest_index = rrt.find_nearest(&rand_point);
            let nearest_node = &rrt.nodes[nearest_index];
            let new_point = rrt.steer(&nearest_node.point, &rand_point);

            if rrt.is_collision_free(&new_point) {
                rrt.add_node(new_point, nearest_index);
            }

            if new_point.distance(&rrt.goal) < rrt.goal_threshold {
                goal_reached = true;
                optimal_path = rrt.trace_path();
                println!("Goal Reached!");
            }
        }

        clear_background(WHITE);

        // Düğümleri ve yolları çiz
        for node in &rrt.nodes {
            if let Some(parent_index) = node.parent {
                let parent_node = &rrt.nodes[parent_index];
                draw_line(
                    node.point.x,
                    node.point.y,
                    parent_node.point.x,
                    parent_node.point.y,
                    1.0,
                    BLUE,
                );
            }
        }

        // Hedefe ulaşıldığında en iyi yolu çiz
        if goal_reached {
            for i in 1..optimal_path.len() {
                let start = optimal_path[i - 1];
                let end = optimal_path[i];
                draw_line(start.x, start.y, end.x, end.y, 2.0, GREEN);
            }
        }

        // Başlangıç ve hedef noktalarını göster
        draw_circle(rrt.nodes[0].point.x, rrt.nodes[0].point.y, 5.0, GREEN);
        draw_circle(rrt.goal.x, rrt.goal.y, 5.0, RED);

        next_frame().await;
    }
}
