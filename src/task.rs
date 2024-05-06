use core::sync::atomic::AtomicU32;
use core::future::Future;
use core::ptr::NonNull;
use core::task::{Context, Poll};
use core::pin::Pin;

use alloc::boxed::Box;
use alloc::collections::VecDeque;
use alloc::sync::Arc;
use spin::Mutex;

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
                            log::debug!("read task");
                            task.driver.register_readwaker(waker);
                        },
                        2 => {
                            // TaskIOType::Write
                            log::debug!("write task");
                            task.driver.register_writewaker(waker);
                        },
                        _ => {
                            log::debug!("Error task iotype {}", task.iotype.load(core::sync::atomic::Ordering::Relaxed))
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
    pub fn as_ref(self: Arc<Self>) -> TaskRef {
        unsafe { TaskRef::from_ptr(Arc::into_raw(self))}
    }

    /// 
    pub fn from_ref(task_ref: TaskRef) -> Arc<Self> {
        let raw_ptr = task_ref.as_task_raw_ptr();
        unsafe { Arc::from_raw(raw_ptr) }
    }
}

/// Wake a task by a 'TaskRef'
#[inline(always)]
pub fn wake_task(task_ref: TaskRef) {
    unsafe {
        // 修改 Task 状态，等到接收到串口中断时，执行器会执行里面现有的就绪 Future
        let raw_ptr = task_ref.as_task_raw_ptr();
        (*raw_ptr).state.store(TaskState::Ready as u32, core::sync::atomic::Ordering::Relaxed);
        log::debug!("wake_task, the tasks' state is {} (Ready == 1)", (*raw_ptr).state.load(core::sync::atomic::Ordering::Relaxed));
        // let _iotype = (*raw_ptr).iotype.load( core::sync::atomic::Ordering::Relaxed);
    }
}


#[derive(Default)]
pub struct Executor {
    tasks: Mutex<VecDeque<Arc<Task>>>,
}

impl Executor {
    // fn add_task(&self, task_ref: TaskRef) {
    //     self.tasks.lock().push_back(Task::from_ref(task_ref));
    // }

    pub fn is_empty(&self) -> bool {
        self.tasks.lock().is_empty()
    }

    pub fn push_task(&self, task: Arc<Task>) {
        self.tasks.lock().push_back(task);
    }

    pub fn pop_runnable_task(&self) -> Option<Arc<Task>> {
        let mut tasks = self.tasks.lock();
        for _ in 0..tasks.len() {
            let task = tasks.pop_front().unwrap();
            let tstate = task.state.load(core::sync::atomic::Ordering::Relaxed);
            if tstate == TaskState::Ready as u32 {
                return Some(task)
            }
            tasks.push_back(task);
        }
        None
    }

    // pub fn spawn(&self, future: impl Future<Output = i32> + 'static + Send + Sync) 

    pub fn run_until_idle(&self) -> bool {
        while let Some(task) = self.pop_runnable_task() {
            task.state.store(TaskState::Pending as u32, core::sync::atomic::Ordering::Relaxed);
            let task_ref = task.clone().as_ref();
            if task_ref.poll() == Poll::Pending {
                self.push_task(task)
            }
        }
        !self.is_empty()
    }
}