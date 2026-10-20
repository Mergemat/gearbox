use std::cell::RefCell;
use std::rc::Rc;

pub struct Memo<K, V>(RefCell<Option<(K, Rc<V>)>>);

impl<K, V> Default for Memo<K, V> {
    fn default() -> Self {
        Self(RefCell::new(None))
    }
}

impl<K: PartialEq, V> Memo<K, V> {
    pub fn get(&self, key: K, make: impl FnOnce() -> V) -> Rc<V> {
        if let Some((k, v)) = self.0.borrow().as_ref()
            && *k == key
        {
            return v.clone();
        }
        let v = Rc::new(make());
        *self.0.borrow_mut() = Some((key, v.clone()));
        v
    }
}
