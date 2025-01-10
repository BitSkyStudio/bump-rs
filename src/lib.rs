use std::collections::{HashMap, HashSet};
use generational_arena::{Arena, Index};
use nalgebra::Vector2;

#[derive(Copy, Clone)]
pub enum CollisionResponse{
    Touch,
    Cross,
    Slide,
    Bounce,
}
impl CollisionResponse{
    pub fn response<T: Collidable>(self, world: &World<T>, collision: &Collision, collisions: Vec<Collision>, item: Index, rect: Rectangle, mut goal: Vec2f, filter: impl Fn(Index) -> bool) -> (Vec2f, Vec<Collision>){
        match self{
            CollisionResponse::Touch => {
                (collisions[0].info.touch, Vec::new())
            }
            CollisionResponse::Cross => {
                //original uses another world.project, maybe this causes invalid behaviour?
                (goal, collisions)
            }
            CollisionResponse::Slide => {
                if collision.info.movement.magnitude_squared() != 0.{
                    if collision.info.normal.x != 0.{
                        goal.x = collision.info.touch.x;
                    } else {
                        goal.y = collision.info.touch.y;
                    }
                }
                (goal, world.project(item, Rectangle{
                    position: collision.info.touch,
                    size: rect.size,
                }, goal, filter))
            }
            CollisionResponse::Bounce => {
                goal = if collision.info.movement.magnitude_squared() != 0.{
                    let bn = goal - collision.info.touch;
                    if collision.info.normal.x == 0.{
                        bn.y *= -1.;
                    } else {
                        bn.x *= -1.;
                    }
                    collision.info.touch + bn
                } else {
                    collision.info.touch
                };
                (goal, world.project(item, Rectangle{
                    position: collision.info.touch,
                    size: rect.size,
                }, goal, filter))
            }
        }
    }
}
fn nearest_number(x: f64, a: f64, b: f64) -> f64{
    if (a-x).abs() < (b-x).abs() {a} else {b}
}
const DELTA: f64 = 1e-10;
pub type Vec2f = Vector2<f64>;
#[derive(Copy, Clone)]
pub struct Rectangle{
    pub position: Vec2f,
    pub size: Vec2f,
}
impl Rectangle{
    pub fn end(self) -> Vec2f{
        self.position + self.size
    }
    fn nearest_corner(self, point: Vec2f) -> Vec2f{
        Vec2f::new(nearest_number(point.x, self.position.x, self.end().x), nearest_number(point.y, self.position.y, self.end().y))
    }
    fn mink_diff(self, other: Rectangle) -> Rectangle{
        Rectangle{
            position: Vec2f::new(other.position.x-self.position.x-self.size.x, other.position.y-self.position.y-self.size.y),
            size: Vec2f::new(self.size.x + other.size.x, self.size.y + other.size.y)
        }
    }
    pub fn contains_point(self, point: Vec2f) -> bool{
        point.x - self.position.x > DELTA && point.y - self.position.y > DELTA &&
            self.end().x - point.x > DELTA && self.end().y - point.y > DELTA
    }
    pub fn intersects(self, other: Rectangle) -> bool{
        self.position.x < other.end().x && other.position.x < self.end().x &&
        self.position.y < other.end().y && other.position.y < self.end().y
    }
    fn squared_distance(self, other: Rectangle) -> f64{
        Vec2f::new(
            self.position.x - other.position.x + (self.size.x-other.size.x)/2.,
            self.position.y - other.position.y + (self.size.y-other.size.y)/2.,
        ).magnitude_squared()
    }
    fn segment_intersection_indices(self, p1: Vec2f, p2: Vec2f, mut ti1: f64, mut ti2: f64) -> Option<(f64, f64,Vec2f,Vec2f)>{
        let (dx,dy) = (p2.x-p1.x, p2.y-p1.y);
        let mut n1 = Vec2f::new(0., 0.);
        let mut n2 = Vec2f::new(0., 0.);
        for (n,p,q) in [
            (Vec2f::new(-1.,0.),-dx, p1.x-self.position.x),
            (Vec2f::new(1.,0.),dx, self.end().x - p1.x),
            (Vec2f::new(0., -1.),-dy, p1.y-self.position.y),
            (Vec2f::new(0., 1.),dy, self.end().y - p1.y),
        ]{
            if p == 0. {
                if q <= 0. {return None;}
            } else {
                let r = q / p;
                if p < 0.{
                    if r > ti2 {return None;}
                    else if r > ti1 {
                        ti1 = r;
                        n1 = n;
                    }
                } else {
                    if r < ti1 {return None;}
                    else if r < ti2 {
                        ti2 = r;
                        n2 = n;
                    }
                }
            }
        }
        Some((ti1, ti2, n1, n2))
    }
    fn detect_collision(self, other: Rectangle, goal: Vec2f) -> Option<RectCollision>{
        let d = Vec2f::new(goal.x - self.position.x, goal.y - self.position.y);
        let diff_rect = self.mink_diff(other);
        let overlaps;
        let ti;
        let n;
        let t;
        if diff_rect.contains_point(Vec2f::new(0.0, 0.0)){
            let corner = diff_rect.nearest_corner(Vec2f::new(0.0, 0.0));
            let (wi, hi) = (self.size.x.min(corner.x.abs()), self.size.y.min(corner.y.abs()));
            ti = -wi * hi;
            overlaps = true;
            if d.magnitude_squared() == 0. {
                let p = diff_rect.nearest_corner(Vec2f::new(0., 0.));
                if p.x.abs() < p.y.abs() {
                    p.y = 0.;
                } else {
                    p.x = 0.;
                }
                n = Vec2f::new(p.x.signum(), p.y.signum());
                t = Vec2f::new(self.position.x + p.x, self.position.y + p.y);
            } else {
                if let Some((ti1, _, n2, _)) = diff_rect.segment_intersection_indices(Vec2f::new(0., 0.), d, -f64::INFINITY, 1.){
                    n = n2;
                    t = Vec2f::new(self.position.x + d.x * ti1, self.position.y + d.y * ti1);
                } else {
                    return None;
                }
            }
        } else {
            if let Some((ti1, ti2, n1, _)) = diff_rect.segment_intersection_indices(Vec2f::new(0., 0.), d, -f64::INFINITY, f64::INFINITY) {
                if ti1 < 1. && (ti1-ti2).abs() >= DELTA && (0. < ti1 + DELTA || 0. == ti1 && ti2 > 0.){
                    ti = ti1;
                    n = n1;
                    overlaps = false;
                    t = Vec2f::new(self.position.x + d.x * ti, self.position.y + d.y * ti);
                } else {
                    return None;
                }
            } else {
                return None;
            }
        }
        Some(RectCollision{
            overlaps,
            ti,
            movement: d,
            normal: n,
            touch: t,
            item_rect: self,
            other_rect: other,
        })
    }
}
#[derive(Copy, Clone)]
pub struct RectCollision{
    pub overlaps: bool,
    pub ti: f64,
    pub movement: Vec2f,
    pub normal: Vec2f,
    pub touch: Vec2f,
    pub item_rect: Rectangle,
    pub other_rect: Rectangle,
}
#[derive(Copy, Clone)]
pub struct Collision{
    pub info: RectCollision,
    pub item: Index,
    pub other: Index,
    pub response_type: CollisionResponse,
}
pub struct World<T: Collidable>{
    items: Arena<WorldItem<T>>,
    cells: HashMap<CellIndex, Vec<Index>>,
    grid: CellGrid,
}
impl<T: Collidable> World<T>{
    pub fn new() -> Self{
        Self::with_cell_size(64.)
    }
    pub fn with_cell_size(cell_size: f64) -> Self{
        World{
            items: Arena::new(),
            cells: HashMap::new(),
            grid: CellGrid{
                cell_size,
            },
        }
    }
    pub fn get_item(&self, item: Index) -> Option<&T>{
        self.items.get(item).map(|i|i.data)
    }
    pub fn get_rect(&self, item: Index) -> Option<Rectangle>{
        self.items.get(item).map(|i|i.rect)
    }
    pub fn get_item_mut(&mut self, item: Index) -> Option<&mut T>{
        self.items.get_mut(item).map(|i|i.data)
    }
    pub fn contains(&self, item: Index) -> bool{
        self.items.contains(item)
    }
    pub fn query_rect(&self, rect: Rectangle) -> Vec<Index> {
        let mut query = Vec::new();
        for cell_index in self.grid.cell_rect(rect).into_iter() {
            if let Some(cell) = self.cells.get(&cell_index) {
                for other in cell {
                    if rect.intersects(self.get_rect(*other).unwrap()){
                        query.push(*other);
                    }
                }
            }
        }
        query
    }
    pub fn query_point(&self, point: Vec2f) -> Vec<Index>{
        let mut query = Vec::new();
        if let Some(cell) = self.cells.get(&self.grid.to_cell(point)){
            for other in cell {
                if self.get_rect(*other).unwrap().contains_point(point) {
                    query.push(*other);
                }
            }
        }
        query
    }
    pub fn query_line_segment(&self, start: Vec2f, end: Vec2f) -> Vec<LineSegmentQuery>{
        let mut query = Vec::new();
        let mut visited = HashSet::new();
        let d = end - start;
        self.grid.traverse(start, end, |cell_index|{
            if let Some(cell) = self.cells.get(&cell_index){
                for other in cell {
                    if visited.insert(*other) {
                        let rect = self.get_rect(*other).unwrap();
                        if let Some((ti1, ti2, _, _)) = rect.segment_intersection_indices(start, end, 0., 1.){
                            if (0. < ti1 && ti1 < 1.) || (0. < ti2 && ti2 < 1.){
                                let (tii0, tii1, _, _) = rect.segment_intersection_indices(start, end, -f64::INFINITY, f64::INFINITY).unwrap();
                                query.push((LineSegmentQuery{
                                    item: *other,
                                    ti1,
                                    ti2,
                                    p1: start + d * ti1,
                                    p2: start + d * ti2,
                                }, tii0.min(tii1)));
                            }
                        }
                    }
                }
            }
        });
        query.sort_by(|q1, q2|q1.1.total_cmp(&q2.1));
        query.into_iter().map(|q|q.0).collect()
    }
    pub fn add(&mut self, item: T, rect: Rectangle) -> Index{
        let index = self.items.insert(WorldItem{
            data: item,
            rect,
        });
        for cell in self.grid.cell_rect(rect).into_iter(){
            self.add_item_to_cell(cell, index);
        }
        index
    }
    pub fn remove(&mut self, index: Index) -> Option<(T, Rectangle)>{
        match self.items.remove(index){
            Some(world_item) => {
                for cell in self.grid.cell_rect(world_item.rect).into_iter(){
                    self.remove_item_from_cell(cell, index);
                }
                Some((world_item.data, world_item.rect))
            }
            None => None,
        }
    }
    pub fn update(&mut self, index: Index, new_rect: Rectangle) -> Option<Rectangle>{
        let old_rect = self.get_rect(*index)?;
        let new_rect_cells = self.grid.cell_rect(new_rect);
        let old_rect_cells = self.grid.cell_rect(old_rect);
        if new_rect_cells != old_rect_cells{
            //todo: unoptimal
            let new_cells = new_rect_cells.into_iter().collect::<HashSet<_>>();
            let old_cells = old_rect_cells.into_iter().collect::<HashSet<_>>();
            for add_cells in new_cells.difference(&old_cells){
                self.add_item_to_cell(*add_cells, *index);
            }
            for remove_cells in old_cells.difference(&new_cells){
                self.remove_item_from_cell(*remove_cells, *index);
            }
        }
        self.items.get_mut(index).unwrap().rect = new_rect;
        Some(old_rect)
    }
    pub fn check(&self, index: Index, mut goal: Vec2f) -> Option<(Vec2f, Vec<Collision>)>{
        let mut visited = HashSet::new();
        let rect = self.get_rect(index)?;
        let mut collisions = Vec::new();
        let mut projected_collisions = self.project(index, rect, goal, |index|!visited.contains(&index));
        while projected_collisions.len() > 0{
            let collision = projected_collisions.remove(0);
            collisions.push(collision);
            visited.insert(collision.other);
            (goal, projected_collisions) = collision.response_type.response(self, &collision, projected_collisions, index, rect, goal, |index|!visited.contains(&index));
        }
        Some((goal, collisions))
    }
    pub fn move_item(&mut self, index: Index, goal: Vec2f) -> Option<(Vec2f, Vec<Collision>)>{
        let checked = self.check(index, goal)?;
        self.update(index, Rectangle{
            position: checked.0,
            size: self.get_rect(index).unwrap().size,
        });
        Some(checked)
    }
    fn add_item_to_cell(&mut self, cell: CellIndex, item: Index) {
        self.cells.entry(cell).or_insert_with(Vec::new).push(item);
    }
    fn remove_item_from_cell(&mut self, cell: CellIndex, item: Index){
        let remove = if let Some(cell) = self.cells.get_mut(&cell){
            let index = cell.iter().position(|&x| x == item).expect("item not found");
            cell.remove(index);
            cell.is_empty()
        } else {
            panic!("Tried to remove item from cell that does not exist");
        };
        if remove {
            self.cells.remove(&cell);
        }
    }
    fn project(&self, item: Index, rect: Rectangle, goal: Vec2f, filter: impl Fn(Index) -> bool) -> Vec<Collision>{
        let mut collisions = Vec::new();
        let mut visited = HashSet::new();
        visited.insert(item);
        let cr = self.grid.cell_rect(Rectangle{
            position: Vec2f::new(goal.x.min(rect.position.x), goal.y.min(rect.position.y)),
            size: Vec2f::new((goal.x+rect.size.x).max(rect.end().x), (goal.y+rect.size.y).max(rect.end().y)),
        });
        for cell_index in cr.into_iter(){
            if let Some(cell) = self.cells.get(&cell_index){
                for other in cell{
                    if visited.insert(*other) && filter(*other){
                        let response = self.items.get(item).unwrap().data.collision_response(self.items.get(*other).unwrap());
                        if let Some(response) = response {
                            let other_rect = self.items.get(*other).unwrap().rect;
                            if let Some(collision) = rect.detect_collision(other_rect, goal) {
                                collisions.push(Collision {
                                    info: collision,
                                    item,
                                    other: *other,
                                    response_type: response,
                                })
                            }
                        }
                    }
                }
            }
        }
        collisions.sort_by(|a, b| {
            if a.info.ti == b.info.ti{
                let ad = a.info.item_rect.squared_distance(a.info.other_rect);
                let bd = a.info.item_rect.squared_distance(b.info.other_rect);
                ad.total_cmp(&bd)
            } else {
                a.info.ti.total_cmp(&b.info.ti)
            }
        });
        collisions
    }
}
#[derive(Copy, Clone)]
pub struct LineSegmentQuery{
    item: Index,
    ti1: f64,
    ti2: f64,
    p1: Vec2f,
    p2: Vec2f,
}
#[derive(Copy, Clone)]
struct CellGrid{
    cell_size: f64,
}
impl CellGrid{
    fn to_world(self, cell: CellIndex) -> Vec2f{
        Vec2f::new((cell.x-1) as f64 * self.cell_size, (cell.y-1) as f64 * self.cell_size)
    }
    fn to_cell(self, pos: Vec2f) -> CellIndex{
        CellIndex{
            x: (pos.x / self.cell_size).floor() as i32 + 1,
            y: (pos.y / self.cell_size).floor() as i32 + 1,
        }
    }
    fn traverse_init_step(self, ct: f64, t1: f64, t2: f64) -> (i32, f64, f64){
        let v = t2 - t1;
        if v > 0. {
            (1, self.cell_size / v, ((ct + v) * self.cell_size - t1) / v)
        } else if v < 0. {
            (-1, -self.cell_size / v, ((ct + v - 1.) * self.cell_size - t1) / v)
        } else {
            (0, f64::INFINITY, f64::INFINITY)
        }
    }
    fn traverse(self, p1: Vec2f, p2: Vec2f, f: impl Fn(CellIndex)){
        let mut c1 = self.to_cell(p1);
        let c2 = self.to_cell(p2);
        let (step_x, dx, mut tx) = self.traverse_init_step(c1.x as f64, p1.x, p2.x);
        let (step_y, dy, mut ty) = self.traverse_init_step(c1.y as f64, p1.y, p2.y);
        f(c1);
        while (c1.x-c2.x).abs() + (c1.y-c2.y).abs() > 1{
            if tx < ty{
                tx += dx;
                c1.x += step_x;
            } else {
                if tx == ty{
                    f(CellIndex{x: c1.x + step_x, y: c1.y});
                }
                ty += dy;
                c1.y += step_y;
            }
            f(c1);
        }
        if c1 != c2 {
            f(c2);
        }
    }
    fn cell_rect(self, rect: Rectangle) -> CellRectangle{
        let c = self.to_cell(rect.position);
        let s = CellIndex{x: (rect.end().x/self.cell_size).ceil() as i32, y: (rect.end().y/self.cell_size).ceil() as i32};
        CellRectangle{
            position: c,
            size: CellIndex{x: s.x-c.x, y: s.y-c.y},
        }
    }
}
#[derive(Copy, Clone, PartialEq, Eq, Hash)]
struct CellIndex{
    x: i32,
    y: i32,
}
#[derive(Copy, Clone, PartialEq)]
struct CellRectangle{
    position: CellIndex,
    size: CellIndex,
}
impl IntoIterator for CellRectangle{
    type Item = CellIndex;
    type IntoIter = CellRectangleIter;
    fn into_iter(self) -> Self::IntoIter {
        CellRectangleIter{
            rect: self,
            x: self.position.x,
            y: self.position.y,
        }
    }
}
struct CellRectangleIter{
    rect: CellRectangle,
    x: i32,
    y: i32,
}
impl Iterator for CellRectangleIter{
    type Item = CellIndex;
    fn next(&mut self) -> Option<CellIndex> {
        if self.y >= self.rect.end().y{
            return None;
        }
        let return_value = CellIndex{x: self.x, y: self.y};
        self.x += 1;
        if self.x >= self.rect.end().x{
            self.x = self.rect.position.x;
            self.y += 1;
        }
        Some(return_value)
    }
}
impl CellRectangle{
    fn end(self) -> CellIndex{
        CellIndex{
            x: self.position.x + self.size.x,
            y: self.position.y + self.size.y,
        }
    }
}
struct WorldItem<T: Collidable>{
    data: T,
    rect: Rectangle,
}
pub trait Collidable{
    fn collision_response(&self, other: &Self) -> Option<CollisionResponse>;
}