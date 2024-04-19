use core::sync::atomic::AtomicU32;
use core::future::Future;
use core::ptr::NonNull;
use core::task::{Context, Poll};
use core::pin::Pin;

use alloc::boxed::Box;
use alloc::sync::Arc;

use crossbeam::atomic::AtomicCell;

use crate::serials::AsyncSerial;
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

#[repr(u32)]
pub enum TaskIOType {
    Read = 1 << 0,
    Write = 1 << 1,
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
                    task.state.store(TaskState::Pending as u32, core::sync::atomic::Ordering::Relaxed);
                    match task.iotype.load(core::sync::atomic::Ordering::Relaxed) {
                        1 => {
                            // TaskIOType::Read
                            task.driver.register_readwaker(waker);
                        },
                        2 => {
                            // TaskIOType::Write
                            task.driver.register_writewaker(waker);
                        },
                        _ => {
                            log::debug!("Error task iotype")
                        }
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
    /// driver
    pub driver: Arc<AsyncSerial>,
    /// IO Type
    pub iotype: AtomicU32,
}

impl Task {
    /// Create a new Task 
    pub fn new(
        fut: Pin<Box<dyn Future<Output = i32> + 'static + Send + Sync>>,
        driver: Arc<AsyncSerial>,
        iotype: TaskIOType,
    ) -> TaskRef {
        let task = Arc::new(Self{
            state: AtomicU32::new(TaskState::Ready as u32),
            fut: AtomicCell::new(fut),
            driver,
            iotype: AtomicU32::new(iotype as u32)
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
        (*raw_ptr).state.store(TaskState::Running as u32, core::sync::atomic::Ordering::Relaxed);
        log::debug!("wake_task {}", (*raw_ptr).state.load(core::sync::atomic::Ordering::Relaxed));
        let iotype = (*raw_ptr).iotype.load( core::sync::atomic::Ordering::Relaxed);
        // 模拟放到执行器中执行
        match task_ref.poll() {
            Poll::Ready(a) => {
                log::debug!("{} task finished {}",iotype, a)
            },
            Poll::Pending => {
                log::debug!("{} task still pending", iotype)
            }
        }
    }
}
