use core::sync::atomic::AtomicU32;
use core::future::Future;
use core::ptr::NonNull;
use core::task::{Context, Poll};
use core::pin::Pin;

use alloc::boxed::Box;
use alloc::sync::Arc;

use crossbeam::atomic::AtomicCell;

use crate::waker::from_task;

/// 
#[repr(u32)]
pub enum TaskState {
    ///
    Ready = 1 << 0,
    ///
    Running = 1 << 1,
    ///
    Pending = 1 << 2,
}

/// The pointer of 'Task'
#[derive(Debug, Clone)]
pub struct TaskRef {
    ptr: NonNull<Task>,
}

unsafe impl Send for TaskRef {}
unsafe impl Sync for TaskRef {}

impl TaskRef {
    /// From a 'TaskRef' to a 'Task' raw_pointer
    pub fn as_task_raw_ptr(&self) -> *const Task {
        self.ptr.as_ptr()
    }

    /// From a 'Task' raw_pointer to a 'TaskRef'
    pub(crate) unsafe fn from_ptr(ptr: *const Task) -> Self {
        Self {
            ptr: NonNull::new(ptr as *mut Task).unwrap(),
        }
    }

    /// poll the task
    #[inline(always)]
    pub fn poll(self) -> Poll<i32> {
        unsafe {
            let waker = from_task(self.clone());
            let mut cx: Context<'_> = Context::from_waker(&waker);
            let task = Task::from_ref(self);
            let future = &mut *task.fut.as_ptr();
            match future.as_mut().poll(&mut cx) {
                Poll::Ready(res) => Poll::Ready(res),
                Poll::Pending => {
                    if task.state.load(core::sync::atomic::Ordering::Relaxed) == TaskState::Ready as u32{
                        task.state.store(TaskState::Pending as u32, core::sync::atomic::Ordering::Relaxed);
                    }
                    log::debug!("pending {}", task.state.load(core::sync::atomic::Ordering::Relaxed));
                    Poll::Pending
                },
            }
        }
    }
}


pub struct Task {

    /// detail value shown in 'TaskRef'
    pub(crate) state: AtomicU32,
    /// The task future
    pub fut: AtomicCell<Pin<Box<dyn Future<Output = i32> + 'static + Send + Sync>>>,
}

impl Task {
    /// Create a new Task 
    pub fn new(
        fut: Pin<Box<dyn Future<Output = i32> + 'static + Send + Sync>>
    ) -> TaskRef {
        let task = Arc::new(Self{
            state: AtomicU32::new(TaskState::Ready as u32),
            fut: AtomicCell::new(fut),
        });
        task.as_ref()
    }

    /// 
    fn as_ref(self: Arc<Self>) -> TaskRef {
        unsafe { TaskRef::from_ptr(Arc::into_raw(self))}
    }

    /// 
    fn from_ref(task_ref: TaskRef) -> Arc<Self> {
        let raw_ptr = task_ref.as_task_raw_ptr();
        unsafe { Arc::from_raw(raw_ptr) }
    }
}

/// Wake a task by a 'TaskRef'
#[inline(always)]
pub fn wake_task(task_ref: TaskRef) {
    unsafe {
        let raw_ptr = task_ref.as_task_raw_ptr();
        (*raw_ptr).state.store(TaskState::Ready as u32, core::sync::atomic::Ordering::Relaxed);
        log::debug!("wake_task {}", (*raw_ptr).state.load(core::sync::atomic::Ordering::Relaxed));
    }
}
