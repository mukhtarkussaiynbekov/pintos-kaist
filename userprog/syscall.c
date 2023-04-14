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
// void close (int fd);

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
	// printf("syscall_handler = %d\n", f->R.rax);
	switch (f->R.rax) {
		case SYS_HALT:
			halt();
			break;
		case SYS_EXIT:
			// printf("Rdi = %d\n", f->R.rdi);
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
	}
}

void
validate_ptr (void *p) {
	if (!p || is_kernel_vaddr (p) || !pml4_get_page (thread_current ()->pml4, p)) {
		// printf ("validation failed\n");
		exit (-1);
	}
}

void
halt (void) {
	power_off ();
}

void
exit (int status) {
	// printf("setting status %d\n", status);
	set_status (thread_current (), status);
	// if (thread_current()->self_status)
	// 	printf("already set status %d -> %d\n", status, thread_current()->self_status->exit_status);
	printf ("%s: exit(%d)\n", thread_name (), status);

	thread_exit ();
}

int
fork (const char *thread_name, struct intr_frame *if_) {
	// validate start and end of pointer are valid
	validate_ptr (thread_name);
	validate_ptr (thread_name + strlen (thread_name));
	// printf ("fork success validation of pointers\n");
	return process_fork (thread_name, if_);
}

int
exec (const char *cmd_line, struct intr_frame *if_) {
	// validate start and end of pointer are valid
	validate_ptr (cmd_line);
	validate_ptr (cmd_line + strlen (cmd_line));
	/* Create a new child to execute CMD_LINE. */
	char tname[fstarg_length (cmd_line) + 1];
	// char *tname = malloc (fstarg_length (cmd_line) + 1);
	strcpy_fstarg (tname, cmd_line);
	// printf ("tname = %s\n", tname);
	tid_t child_tid;
	if ((child_tid = process_fork (tname, if_))) {
		// printf ("child_tid = %d\n", child_tid);
		struct status *child_status = get_child_status (child_tid, &thread_current ()->children);
		// struct thread *child = get_thread_by_tid (child_tid);
		if (child_status && child_status->self)
			sema_down (&child_status->self->load_sema);
		return child_tid;
		// return wait(child_tid);
	} else {
		// printf ("child process is about to call exec\n");
		// free (tname);
		return process_exec (cmd_line);
	}
}

int
wait (int pid) {
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
	struct thread *curr = thread_current ();
	int open_status = -1;
	lock_acquire (&fs_lock);
	for (int i = 2; i < FDT_SIZE; i++) {
		if (!curr->fdt[i]) {
			curr->fdt[i] = filesys_open (file);
			if (curr->fdt[i])
				open_status = i;
			break;
		}
	}
	lock_release (&fs_lock);
	return open_status;
}

int
filesize (int fd) {
	struct thread *curr = thread_current ();
	if (!curr->fdt[fd]) return -1;
	lock_acquire (&fs_lock);
	int file_size = file_length (curr->fdt[fd]);
	lock_release (&fs_lock);
	return file_size;
}

int
read (int fd, void *buffer, unsigned size) {
	// validate start and end of pointer are valid
	validate_ptr (buffer);
	validate_ptr (buffer + size - 1);
	struct thread *curr = thread_current ();
	int read_count = size;
	lock_acquire (&fs_lock);
	if (fd == 0) {
		for (unsigned i = 0; i < size; i++)
			((char *) buffer)[i] = input_getc ();
	} else {
		if (fd < 0 || fd >= FDT_SIZE || !curr->fdt[fd]) read_count = -1;
		else read_count = file_read (curr->fdt[fd], buffer, size);
	}
	lock_release (&fs_lock);
	return read_count;
}

int
write (int fd, const void *buffer, unsigned size) {
	// validate start and end of pointer are valid
	validate_ptr (buffer);
	validate_ptr (buffer + size - 1);
	struct thread *curr = thread_current ();
	int written_count = size;
	lock_acquire (&fs_lock);
	if (fd == 1) {
		putbuf (buffer, size);
	} else {
		if (fd < 0 || fd >= FDT_SIZE || !curr->fdt[fd]) written_count = -1;
		else written_count = file_write (curr->fdt[fd], buffer, size);
	}
	lock_release (&fs_lock);
	return written_count;
}

void 
seek (int fd, unsigned position) {
	struct thread *curr = thread_current ();
	if (fd < 0 || fd >= FDT_SIZE || !curr->fdt[fd]) return;
	lock_acquire (&fs_lock);
	file_seek (curr->fdt[fd], position);
	lock_release (&fs_lock);
}

unsigned
tell (int fd) {
	struct thread *curr = thread_current ();
	if (fd < 0 || fd >= FDT_SIZE || !curr->fdt[fd]) return 0;
	lock_acquire (&fs_lock);
	unsigned ftell = file_tell (curr->fdt[fd]);
	lock_release (&fs_lock);
	return ftell;
}

void 
close (int fd) {
	struct thread *curr = thread_current ();
	if (fd < 0 || fd >= FDT_SIZE || !curr->fdt[fd]) return;
	lock_acquire (&fs_lock);
	file_close (curr->fdt[fd]);
	curr->fdt[fd] = NULL;
	lock_release (&fs_lock);
}
