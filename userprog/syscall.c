#include "userprog/syscall.h"
#include "userprog/process.h"
#include <stdio.h>
#include <syscall-nr.h>
#include <list.h>
#include <string.h>
#include "threads/interrupt.h"
#include "threads/thread.h"
#include "threads/synch.h"
#include "threads/loader.h"
#include "userprog/gdt.h"
#include "threads/flags.h"
#include "intrinsic.h"
#include "threads/init.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "devices/input.h"
#include "threads/malloc.h"

void syscall_entry (void);
void syscall_handler (struct intr_frame *);

struct lock fs_lock;	// Global File System Lock to synchronize file system calls
void validate_ptr (void *p);
void halt (void);
void exit (int status);
int fork (const char *thread_name, struct intr_frame *if_);
int exec (const char *cmd_line, struct intr_frame *if_);
int wait (int pid);
bool create (const char *file, unsigned initial_size);
bool remove (const char *file);
int open (const char *file);
int filesize (int fd);
int read (int fd, void *buffer, unsigned size);
int write (int fd, const void *buffer, unsigned size);
void seek (int fd, unsigned position);
unsigned tell (int fd);
void close (int fd);
int dup2(int oldfd, int newfd);
void close_without_fs_lock (int fd);
bool init_fds (void);

/* System call.
 *
 * Previously system call services was handled by the interrupt handler
 * (e.g. int 0x80 in linux). However, in x86-64, the manufacturer supplies
 * efficient path for requesting the system call, the `syscall` instruction.
 *
 * The syscall instruction works by reading the values from the the Model
 * Specific Register (MSR). For the details, see the manual. */

#define MSR_STAR 0xc0000081         /* Segment selector msr */
#define MSR_LSTAR 0xc0000082        /* Long mode SYSCALL target */
#define MSR_SYSCALL_MASK 0xc0000084 /* Mask for the eflags */

void
syscall_init (void) {
	write_msr(MSR_STAR, ((uint64_t)SEL_UCSEG - 0x10) << 48  |
			((uint64_t)SEL_KCSEG) << 32);
	write_msr(MSR_LSTAR, (uint64_t) syscall_entry);

	/* The interrupt service rountine should not serve any interrupts
	 * until the syscall_entry swaps the userland stack to the kernel
	 * mode stack. Therefore, we masked the FLAG_FL. */
	write_msr(MSR_SYSCALL_MASK,
			FLAG_IF | FLAG_TF | FLAG_DF | FLAG_IOPL | FLAG_AC | FLAG_NT);
		
	lock_init (&fs_lock);
}

/* The main system call interface */
void
syscall_handler (struct intr_frame *f) {
	switch (f->R.rax) {
		case SYS_HALT:
			halt();
			break;
		case SYS_EXIT:
			exit (f->R.rdi);
			break;
		case SYS_WAIT:
			f->R.rax = wait (f->R.rdi);
			break;
		case SYS_FORK:
			f->R.rax = fork (f->R.rdi, f);
			break;
		case SYS_EXEC:
			f->R.rax = exec (f->R.rdi, f);
			break;
		case SYS_CREATE:
			f->R.rax = create (f->R.rdi, f->R.rsi);
			break;
		case SYS_REMOVE:
			f->R.rax = remove (f->R.rdi);
			break;
		case SYS_OPEN:
			f->R.rax = open (f->R.rdi);
			break;
		case SYS_FILESIZE:
			f->R.rax = filesize (f->R.rdi);
			break;
		case SYS_READ:
			f->R.rax = read (f->R.rdi, f->R.rsi, f->R.rdx);
			break;
		case SYS_WRITE:
			f->R.rax = write (f->R.rdi, f->R.rsi, f->R.rdx);
			break;
		case SYS_SEEK:
			seek (f->R.rdi, f->R.rsi);
			break;
		case SYS_TELL:
			f->R.rax = tell (f->R.rdi);
			break;
		case SYS_CLOSE:
			close (f->R.rdi);
			break;
		case SYS_DUP2:
			f->R.rax = dup2 (f->R.rdi, f->R.rsi);
			break;
	}
}

bool 
init_fds () {
	struct thread *t = thread_current ();
	if (!t->stdin) {
		struct fdt_entry *stdin = malloc (sizeof (struct fdt_entry));
		if (!stdin) return false;
		stdin->fd = STDIN_FILENO;
		stdin->file = &t->stdin;
		list_insert_ordered (&t->fdt, &stdin->elem, fdt_entry_fd_less, NULL);
		t->stdin = true;
	}
	if (!t->stdout) {
		struct fdt_entry *stdout = malloc (sizeof (struct fdt_entry));
		if (!stdout) return false;
		stdout->fd = STDOUT_FILENO;
		stdout->file = &t->stdout;
		list_insert_ordered (&t->fdt, &stdout->elem, fdt_entry_fd_less, NULL);
		t->stdout = true;
	}
	return true;
}

void
validate_ptr (void *p) {
	if (!p || is_kernel_vaddr (p) || !pml4_get_page (thread_current ()->pml4, p)) {
		exit (-1);
	}
}

void
halt (void) {
	power_off ();
}

void
exit (int status) {
	set_status (thread_current (), status);
	printf ("%s: exit(%d)\n", thread_name (), status);

	thread_exit ();
}

pid_t
fork (const char *thread_name, struct intr_frame *if_) {
	// validate start and end of pointer are valid
	validate_ptr (thread_name);
	validate_ptr (thread_name + strlen (thread_name));
	if (!init_fds ()) return TID_ERROR;
	return process_fork (thread_name, if_);
}

int
exec (const char *cmd_line, struct intr_frame *if_) {
	// validate start and end of pointer are valid
	validate_ptr (cmd_line);
	validate_ptr (cmd_line + strlen (cmd_line));
	char *cmd_copy = malloc (strlen(cmd_line) + 1);
	memcpy (cmd_copy, cmd_line, strlen(cmd_line) + 1);
	return process_exec (cmd_copy, true);
}

int
wait (pid_t pid) {
	return process_wait (pid);
}

bool 
create (const char *file, unsigned initial_size) {
	// validate start and end of pointer are valid
	validate_ptr (file);
	validate_ptr (file + strlen (file));
	lock_acquire (&fs_lock);
	bool status = filesys_create (file, initial_size);
	lock_release (&fs_lock);
	return status;
}

bool 
remove (const char *file) {
	// validate start and end of pointer are valid
	validate_ptr (file);
	validate_ptr (file + strlen (file));
	lock_acquire (&fs_lock);
	bool status = filesys_remove (file);
	lock_release (&fs_lock);
	return status;
}

int 
open (const char *file) {
	// validate start and end of pointer are valid
	validate_ptr (file);
	validate_ptr (file + strlen (file));
	if (!init_fds ()) return -1;
	lock_acquire (&fs_lock);
	struct thread *current = thread_current ();
	int open_status = -1;
	int prevfd = -1;
	struct fdt_entry *cur_entry;
	struct list_elem *cur;
	for (cur = list_begin (&current->fdt); cur != list_end (&current->fdt); cur = list_next (cur)) {
		cur_entry = list_entry (cur, struct fdt_entry, elem);
		if (prevfd + 1 < cur_entry->fd) break;
		prevfd++;
	}
	struct fdt_entry *new_fdt_entry = malloc (sizeof (struct fdt_entry));
	if (new_fdt_entry) {
		struct file *new_file = filesys_open (file);
		if (new_file) {
			new_fdt_entry->fd = prevfd + 1;
			new_fdt_entry->file = new_file;
			list_insert (cur, &new_fdt_entry->elem);
			open_status = new_fdt_entry->fd;
		} else
			free (new_fdt_entry);
	}
	lock_release (&fs_lock);
	return open_status;
}

int
filesize (int fd) {
	lock_acquire (&fs_lock);
	struct thread *curr = thread_current ();
	int file_size;
	struct fdt_entry *target_entry = get_fdt_entry_by_fd (fd);
	if (!target_entry || target_entry->file == &curr->stdin || target_entry->file == &curr->stdout)
		file_size = -1;
	else
		file_size = file_length (target_entry->file);
	lock_release (&fs_lock);
	return file_size;
}

int
read (int fd, void *buffer, unsigned size) {
	// validate start and end of pointer are valid
	validate_ptr (buffer);
	validate_ptr (buffer + size - 1);
	if (!init_fds ()) return -1;
	lock_acquire (&fs_lock);
	struct thread *curr = thread_current ();
	struct fdt_entry *target_entry = get_fdt_entry_by_fd (fd);
	int read_count = size;
	if (!target_entry || target_entry->file == &curr->stdout) 
		read_count = -1;
	else if (target_entry->file == &curr->stdin) {
		for (unsigned i = 0; i < size; i++)
			((char *) buffer)[i] = input_getc ();
	} else
		read_count = file_read (target_entry->file, buffer, size);
	lock_release (&fs_lock);
	return read_count;
}

int
write (int fd, const void *buffer, unsigned size) {
	// validate start and end of pointer are valid
	validate_ptr (buffer);
	validate_ptr (buffer + size - 1);
	if (!init_fds ()) return -1;
	lock_acquire (&fs_lock);
	struct thread *curr = thread_current ();
	struct fdt_entry *target_entry = get_fdt_entry_by_fd (fd);
	int written_count = size;
	if (!target_entry || target_entry->file == &curr->stdin) 
		written_count = -1;
	else if (target_entry->file == &curr->stdout)
		putbuf (buffer, size);
	else
		written_count = file_write (target_entry->file, buffer, size);
	lock_release (&fs_lock);
	return written_count;
}

void 
seek (int fd, unsigned position) {
	lock_acquire (&fs_lock);
	struct thread *curr = thread_current ();
	struct fdt_entry *target_entry = get_fdt_entry_by_fd (fd);
	if (target_entry && !(target_entry->file == &curr->stdin || target_entry->file == &curr->stdout))
		file_seek (target_entry->file, position);
	lock_release (&fs_lock);
}

unsigned
tell (int fd) {
	unsigned ftell;
	lock_acquire (&fs_lock);
	struct thread *curr = thread_current ();
	struct fdt_entry *target_entry = get_fdt_entry_by_fd (fd);
	if (!target_entry || target_entry->file == &curr->stdin || target_entry->file == &curr->stdout)
		ftell = 0;
	else
		ftell = file_tell (target_entry->file);
	lock_release (&fs_lock);
	return ftell;
}

void 
close (int fd) {
	lock_acquire (&fs_lock);
	close_without_fs_lock (fd);
	lock_release (&fs_lock);
}

int 
dup2(int oldfd, int newfd) {
	// verify validity of fds
	if (oldfd < 0 || newfd < 0)
		return -1;

	// close newfd if previously open
	// both closing newfd and setting to oldfd must be performed in one
	// atomic operation, i.e. single lock acquire

	lock_acquire (&fs_lock);
	int resfd = newfd;
	struct fdt_entry *old_fdt_entry = get_fdt_entry_by_fd (oldfd);
	struct fdt_entry *new_fdt_entry = get_fdt_entry_by_fd (newfd);
	if (!old_fdt_entry) resfd = -1;
	else if (oldfd != newfd) {
		close_without_fs_lock (newfd);
		new_fdt_entry = malloc (sizeof (struct fdt_entry));
		if (new_fdt_entry) {
			new_fdt_entry->fd = newfd;
			new_fdt_entry->file = old_fdt_entry->file;
			list_insert_ordered (&thread_current ()->fdt, &new_fdt_entry->elem, fdt_entry_fd_less, NULL);
		} else
			resfd = -1;
	}
	lock_release (&fs_lock);
	return resfd;
}

void
close_without_fs_lock (int fd) {
	if (fd < 0) return;
	struct fdt_entry *target_entry = get_fdt_entry_by_fd (fd);
	if (!target_entry) return;
	struct thread *current = thread_current ();
	bool dup_file = false;
	struct fdt_entry *cur_entry;
	if (!(target_entry->file == &current->stdin || target_entry->file == &current->stdout)) {
		for (struct list_elem *cur_elem = list_begin (&current->fdt); cur_elem != list_end (&current->fdt); cur_elem = list_next (cur_elem)) {
			cur_entry = list_entry (cur_elem, struct fdt_entry, elem);
			if (cur_entry == target_entry) continue;
			if (cur_entry->file == target_entry->file) {
				dup_file = true;
			}
		}
		// duplicate file descriptors point to the same file
		// close file if no duplicate exists
		if (!dup_file)
			file_close (target_entry->file);
	}
	list_remove (&target_entry->elem);
	free (target_entry);
}
