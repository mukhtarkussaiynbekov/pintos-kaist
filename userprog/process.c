#include "userprog/process.h"
#include "userprog/syscall.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "userprog/gdt.h"
#include "userprog/tss.h"
#include "filesys/directory.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/palloc.h"
#include "threads/thread.h"
#include "threads/mmu.h"
#include "threads/vaddr.h"
#include "threads/thread.h"
#include "threads/synch.h"
#include "threads/malloc.h"
#include "intrinsic.h"
#ifdef VM
#include "vm/vm.h"
#endif

static void process_cleanup (void);
static bool load (const char *file_name, char *args, struct intr_frame *if_);
static void initd (void *f_name);
static void __do_fork (void *);
void set_fork_success (struct thread *t, bool success);

/* General process initializer for initd and other process. */
static void
process_init (void) {
	struct thread *current = thread_current ();
}

/* Starts the first userland program, called "initd", loaded from FILE_NAME.
 * The new thread may be scheduled (and may even exit)
 * before process_create_initd() returns. Returns the initd's
 * thread id, or TID_ERROR if the thread cannot be created.
 * Notice that THIS SHOULD BE CALLED ONCE. */
tid_t
process_create_initd (const char *file_name) {
	char *fn_copy;
	tid_t tid;

	/* Make a copy of FILE_NAME.
	 * Otherwise there's a race between the caller and load(). */
	fn_copy = palloc_get_page (0);
	if (fn_copy == NULL)
		return TID_ERROR;
	strlcpy (fn_copy, file_name, PGSIZE);

	/* Create a new thread to execute FILE_NAME. */
	char tname[fstarg_length (fn_copy) + 1];
	strcpy_fstarg (tname, fn_copy);
	tid = thread_create (tname, PRI_DEFAULT, initd, fn_copy);
	if (tid == TID_ERROR)
		palloc_free_page (fn_copy);
	return tid;
}

/* A thread function that launches first user process. */
static void
initd (void *f_name) {
#ifdef VM
	supplemental_page_table_init (&thread_current ()->spt);
#endif

	process_init ();

	if (process_exec (f_name, false) < 0)
		PANIC("Fail to launch initd\n");
	NOT_REACHED ();
}

/* Clones the current process as `name`. Returns the new process's thread id, or
 * TID_ERROR if the thread cannot be created. */
tid_t
process_fork (const char *name, struct intr_frame *if_) {
	/* Clone current thread to new thread.*/
	void **argv = malloc (2 * sizeof (void *));
	argv[0] = thread_current ();
	argv[1] = if_;

	tid_t child_tid = thread_create (name,
			PRI_DEFAULT, __do_fork, argv);
	struct status *child_status;
	struct thread *curr = thread_current ();
	if (child_tid != TID_ERROR) {
		// Wait for child to successfully duplicate resources
		child_status = get_child_status (child_tid, &curr->children);
		sema_down (&child_status->fork_sema);

		// Remove child status from children list and free
		lock_acquire (&child_status->status_lock);
		if (!child_status->fork_success) {
			child_tid = TID_ERROR;
			if (child_status->self)
				child_status->self->self_status = NULL;
			list_remove (&child_status->elem);
			lock_release (&child_status->status_lock);
			free (child_status);
		} else
			lock_release (&child_status->status_lock);
	}
	return child_tid;
}

#ifndef VM
/* Duplicate the parent's address space by passing this function to the
 * pml4_for_each. This is only for the project 2. */
static bool
duplicate_pte (uint64_t *pte, void *va, void *aux) {
	struct thread *current = thread_current ();
	struct thread *parent = (struct thread *) aux;
	void *parent_page;
	void *newpage;
	bool writable;

	/* 1. If the parent_page is kernel page, then return immediately. */
	if (is_kern_pte (pte)) return true;

	/* 2. Resolve VA from the parent's page map level 4. */
	parent_page = pml4_get_page (parent->pml4, va);

	/* 3. Allocate new PAL_USER page for the child and set result to
	 *    NEWPAGE. */
	newpage = palloc_get_page (PAL_USER);
	if (!newpage) return false;

	/* 4. Duplicate parent's page to the new page and
	 *    check whether parent's page is writable or not (set WRITABLE
	 *    according to the result). */
	memcpy(newpage, parent_page, PGSIZE);
	writable = is_writable (pte);

	/* 5. Add new page to child's page table at address VA with WRITABLE
	 *    permission. */
	if (!pml4_set_page (current->pml4, va, newpage, writable)) {
		/* 6. If fail to insert page, do error handling. */
		palloc_free_page (newpage);
		return false;
	}
	return true;
}
#endif

/* A thread function that copies parent's execution context. */
static void
__do_fork (void *aux) {
	struct intr_frame if_;
	struct thread *parent = (struct thread *) ((void **) aux)[0];
	struct thread *current = thread_current ();
	struct intr_frame *parent_if = (struct intr_frame *) ((void **) aux)[1];

	free((void **) aux);	// malloc called in process_fork
	bool succ = true;

	/* 1. Read the cpu context to local stack. */
	memcpy (&if_, parent_if, sizeof (struct intr_frame));
	if_.R.rax = 0; // child process returns 0 when fork is called

	/* 2. Duplicate PT */
	current->pml4 = pml4_create();
	if (current->pml4 == NULL)
		goto error;

	process_activate (current);
#ifdef VM
	supplemental_page_table_init (&current->spt);
	if (!supplemental_page_table_copy (&current->spt, &parent->spt))
		goto error;
#else
	if (!pml4_for_each (parent->pml4, duplicate_pte, parent))
		goto error;
#endif

	bool dup2_called = false;
	struct list_elem *temp_fdt_elem;	// parent's fdt entry
	struct list_elem *prev_fdt_elem;	// current's fdt entry
	struct fdt_entry *temp_fdt_entry;	// parent's fdt entry
	struct fdt_entry *cur_fdt_entry;	// parent's fdt entry
	struct fdt_entry *new_fdt_entry;	// current's fdt entry
	for (struct list_elem *cur_fdt_elem = list_begin (&parent->fdt); cur_fdt_elem != list_end (&parent->fdt); cur_fdt_elem = list_next (cur_fdt_elem)) {
		new_fdt_entry = malloc (sizeof (struct fdt_entry));
		if (!new_fdt_entry)
			goto error;
		list_push_back (&current->fdt, &new_fdt_entry->elem);

		cur_fdt_entry = list_entry (cur_fdt_elem, struct fdt_entry, elem);
		new_fdt_entry->fd = cur_fdt_entry->fd;
		if (cur_fdt_entry->file == &parent->stdin)
			new_fdt_entry->file = &current->stdin;
		else if (cur_fdt_entry->file == &parent->stdout)
			new_fdt_entry->file = &current->stdout;
		else {
			dup2_called = false;
			prev_fdt_elem = list_begin (&current->fdt);
			for (temp_fdt_elem = list_begin (&parent->fdt); temp_fdt_elem != cur_fdt_elem; temp_fdt_elem = list_next (temp_fdt_elem)) {
				temp_fdt_entry = list_entry (temp_fdt_elem, struct fdt_entry, elem);
				if (temp_fdt_entry->file == cur_fdt_entry->file) {
					dup2_called = true;
					break;
				}
				prev_fdt_elem = list_next (prev_fdt_elem);
			}
			if (dup2_called) {
				new_fdt_entry->file = list_entry (prev_fdt_elem, struct fdt_entry, elem)->file;
				continue;
			}
			new_fdt_entry->file = file_duplicate (cur_fdt_entry->file);
			if (!new_fdt_entry->file)
				goto error;
		}
	}
	current->stdin = true;
	current->stdout = true;
	if (parent->exec_file) {
		current->exec_file = file_duplicate (parent->exec_file);
		if (!current->exec_file) {
			goto error;
		}
	}

	process_init ();

	/* Finally, switch to the newly created process. */
	if (succ) {
		set_fork_success (current, true);
		do_iret (&if_);
	}
error:
	set_fork_success (current, false);
	exit (-1);
}

void 
set_fork_success (struct thread *t, bool success) {
	if (t->self_status) {
		t->self_status->fork_success = success;
		sema_up (&t->self_status->fork_sema);
	}
}

/* Switch the current execution context to the f_name.
 * Returns -1 on fail. */
int
process_exec (void *f_name, bool file_malloced) {
	char *file_name = f_name;
	bool success;

	/* We cannot use the intr_frame in the thread structure.
	 * This is because when current thread rescheduled,
	 * it stores the execution information to the member. */
	struct intr_frame _if;
	_if.ds = _if.es = _if.ss = SEL_UDSEG;
	_if.cs = SEL_UCSEG;
	_if.eflags = FLAG_IF | FLAG_MBS;

	/* We first kill the current context */
	if (!file_malloced)
		process_cleanup ();

	/* Second, we separate file name and arguments */
	if (strlen (file_name) >= PGSIZE) {
		// Impose limit on length of command line arguments
		return -1;
	}
	char *file_title, *args;
	file_title = strtok_r (file_name, " ", &args); // argv[0]

	/* And then load the binary */
	success = load (file_title, args, &_if); // args == argv[1~argc-1]

	/* If load failed, quit. */
	if (file_malloced)
		free (file_name);
	else
		palloc_free_page (file_name);
	if (!success)
		return -1;

	/* Start switched process. */
	do_iret (&_if);
	NOT_REACHED ();
}


/* Waits for thread TID to die and returns its exit status.  If
 * it was terminated by the kernel (i.e. killed due to an
 * exception), returns -1.  If TID is invalid or if it was not a
 * child of the calling process, or if process_wait() has already
 * been successfully called for the given TID, returns -1
 * immediately, without waiting.
 *
 * This function will be implemented in problem 2-2.  For now, it
 * does nothing. */
int
process_wait (tid_t child_tid) {
	struct thread *curr = thread_current ();
	struct status *child_status = get_child_status (child_tid, &curr->children);
	if (!child_status) // not direct child or wait has been called previously
		return -1;
	sema_down (&child_status->sema_exit);

	// Remove child status from children list and free
	lock_acquire (&child_status->status_lock);
	int exit_status = child_status->exit_status;
	list_remove (&child_status->elem);
	lock_release (&child_status->status_lock);
	free (child_status);
	return exit_status;
}

/* Exit the process. This function is called by thread_exit (). */
void
process_exit (void) {
	struct thread *curr = thread_current ();
	if (curr->self_status) {
		lock_acquire (&curr->self_status->status_lock);
		// check if parent freed status struct
		// if parent freed, then curr->self_status is set to NULL
		if (curr->self_status) {
			curr->self_status->self = NULL;
			sema_up (&curr->self_status->sema_exit);
			lock_release (&curr->self_status->status_lock);
		}
	}

	struct status *child_status;
	while (!list_empty (&curr->children)) {
		child_status = list_entry (list_pop_front (&curr->children), struct status, elem);
		lock_acquire (&child_status->status_lock);
		if (child_status->self)
			child_status->self->self_status = NULL;
		lock_release (&child_status->status_lock);
		free (child_status);
	}
	
	struct list_elem *temp_fdt_elem;
	struct list_elem *next_fdt_elem;
	struct fdt_entry *temp_fdt_entry;
	struct fdt_entry *cur_fdt_entry;
	for (struct list_elem *cur_fdt_elem = list_begin (&curr->fdt); cur_fdt_elem != list_end (&curr->fdt); cur_fdt_elem = next_fdt_elem) {
		cur_fdt_entry = list_entry (cur_fdt_elem, struct fdt_entry, elem);
		for (temp_fdt_elem = list_next (cur_fdt_elem); temp_fdt_elem != list_end (&curr->fdt); temp_fdt_elem = next_fdt_elem) {
			next_fdt_elem = list_next (temp_fdt_elem);
			temp_fdt_entry = list_entry (temp_fdt_elem, struct fdt_entry, elem);
			if (temp_fdt_entry->file == cur_fdt_entry->file) {
				list_remove (temp_fdt_elem);
				free (temp_fdt_entry);
			}
		}
		next_fdt_elem = list_next (cur_fdt_elem);
		if (cur_fdt_entry->file && !(cur_fdt_entry->file == &curr->stdin || cur_fdt_entry->file == &curr->stdout))
			file_close (cur_fdt_entry->file);
		list_remove (cur_fdt_elem);
		free (cur_fdt_entry);
	}
	
	if (curr->exec_file) {
		file_close (curr->exec_file);
	}

	process_cleanup ();
}

/* Free the current process's resources. */
static void
process_cleanup (void) {
	struct thread *curr = thread_current ();

#ifdef VM
	supplemental_page_table_kill (&curr->spt);
#endif

	uint64_t *pml4;
	/* Destroy the current process's page directory and switch back
	 * to the kernel-only page directory. */
	pml4 = curr->pml4;
	if (pml4 != NULL) {
		/* Correct ordering here is crucial.  We must set
		 * cur->pagedir to NULL before switching page directories,
		 * so that a timer interrupt can't switch back to the
		 * process page directory.  We must activate the base page
		 * directory before destroying the process's page
		 * directory, or our active page directory will be one
		 * that's been freed (and cleared). */
		curr->pml4 = NULL;
		pml4_activate (NULL);
		pml4_destroy (pml4);
	}
}

/* Sets up the CPU for running user code in the nest thread.
 * This function is called on every context switch. */
void
process_activate (struct thread *next) {
	/* Activate thread's page tables. */
	pml4_activate (next->pml4);

	/* Set thread's kernel stack for use in processing interrupts. */
	tss_update (next);
}

/* We load ELF binaries.  The following definitions are taken
 * from the ELF specification, [ELF1], more-or-less verbatim.  */

/* ELF types.  See [ELF1] 1-2. */
#define EI_NIDENT 16

#define PT_NULL    0            /* Ignore. */
#define PT_LOAD    1            /* Loadable segment. */
#define PT_DYNAMIC 2            /* Dynamic linking info. */
#define PT_INTERP  3            /* Name of dynamic loader. */
#define PT_NOTE    4            /* Auxiliary info. */
#define PT_SHLIB   5            /* Reserved. */
#define PT_PHDR    6            /* Program header table. */
#define PT_STACK   0x6474e551   /* Stack segment. */

#define PF_X 1          /* Executable. */
#define PF_W 2          /* Writable. */
#define PF_R 4          /* Readable. */

/* Executable header.  See [ELF1] 1-4 to 1-8.
 * This appears at the very beginning of an ELF binary. */
struct ELF64_hdr {
	unsigned char e_ident[EI_NIDENT];
	uint16_t e_type;
	uint16_t e_machine;
	uint32_t e_version;
	uint64_t e_entry;
	uint64_t e_phoff;
	uint64_t e_shoff;
	uint32_t e_flags;
	uint16_t e_ehsize;
	uint16_t e_phentsize;
	uint16_t e_phnum;
	uint16_t e_shentsize;
	uint16_t e_shnum;
	uint16_t e_shstrndx;
};

struct ELF64_PHDR {
	uint32_t p_type;
	uint32_t p_flags;
	uint64_t p_offset;
	uint64_t p_vaddr;
	uint64_t p_paddr;
	uint64_t p_filesz;
	uint64_t p_memsz;
	uint64_t p_align;
};

/* Abbreviations */
#define ELF ELF64_hdr
#define Phdr ELF64_PHDR

static bool setup_stack (struct intr_frame *if_);
static bool validate_segment (const struct Phdr *, struct file *);
static bool load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes,
		bool writable);

/* Loads an ELF executable from FILE_NAME into the current thread.
 * Stores the executable's entry point into *RIP
 * and its initial stack pointer into *RSP.
 * Returns true if successful, false otherwise. */
static bool
load (const char *file_name, char *args, struct intr_frame *if_) {
	struct thread *t = thread_current ();
	struct ELF ehdr;
	struct file *file = NULL;
	off_t file_ofs;
	bool success = false;
	int i;

	/* Allocate and activate page directory. */
	t->pml4 = pml4_create ();
	if (t->pml4 == NULL)
		goto done;
	process_activate (thread_current ());

	/* Open executable file. */
	file = filesys_open (file_name);
	if (file == NULL) {
		printf ("load: %s: open failed\n", file_name);
		goto done;
	}
	file_deny_write (file);
	if (t->exec_file)
		file_close (t->exec_file);
	t->exec_file = file;

	/* Read and verify executable header. */
	if (file_read (file, &ehdr, sizeof ehdr) != sizeof ehdr
			|| memcmp (ehdr.e_ident, "\177ELF\2\1\1", 7)
			|| ehdr.e_type != 2
			|| ehdr.e_machine != 0x3E // amd64
			|| ehdr.e_version != 1
			|| ehdr.e_phentsize != sizeof (struct Phdr)
			|| ehdr.e_phnum > 1024) {
		printf ("load: %s: error loading executable\n", file_name);
		goto done;
	}

	/* Read program headers. */
	file_ofs = ehdr.e_phoff;
	for (i = 0; i < ehdr.e_phnum; i++) {
		struct Phdr phdr;

		if (file_ofs < 0 || file_ofs > file_length (file))
			goto done;
		file_seek (file, file_ofs);

		if (file_read (file, &phdr, sizeof phdr) != sizeof phdr)
			goto done;
		file_ofs += sizeof phdr;
		switch (phdr.p_type) {
			case PT_NULL:
			case PT_NOTE:
			case PT_PHDR:
			case PT_STACK:
			default:
				/* Ignore this segment. */
				break;
			case PT_DYNAMIC:
			case PT_INTERP:
			case PT_SHLIB:
				goto done;
			case PT_LOAD:
				if (validate_segment (&phdr, file)) {
					bool writable = (phdr.p_flags & PF_W) != 0;
					uint64_t file_page = phdr.p_offset & ~PGMASK;
					uint64_t mem_page = phdr.p_vaddr & ~PGMASK;
					uint64_t page_offset = phdr.p_vaddr & PGMASK;
					uint32_t read_bytes, zero_bytes;
					if (phdr.p_filesz > 0) {
						/* Normal segment.
						 * Read initial part from disk and zero the rest. */
						read_bytes = page_offset + phdr.p_filesz;
						zero_bytes = (ROUND_UP (page_offset + phdr.p_memsz, PGSIZE)
								- read_bytes);
					} else {
						/* Entirely zero.
						 * Don't read anything from disk. */
						read_bytes = 0;
						zero_bytes = ROUND_UP (page_offset + phdr.p_memsz, PGSIZE);
					}
					if (!load_segment (file, file_page, (void *) mem_page,
								read_bytes, zero_bytes, writable))
						goto done;
				}
				else
					goto done;
				break;
		}
	}

	/* Set up stack. */
	if (!setup_stack (if_))
		goto done;

	/* Start address. */
	if_->rip = ehdr.e_entry;

	/* Implement argument passing (see project2/argument_passing.html). */
	uintptr_t end_of_args = if_->rsp; // mark the end of argv
	uint64_t argc = 1; // we at least have argv[0] file name
	size_t args_length = strlen (args);
	size_t arg_size;
	char *arg, *cur_arg_ptr, *save_ptr;
	bool passed_arg = false;
	cur_arg_ptr = args; // args == argv[1~argc-1]

	// push argv[1~argc-1]
	for (cur_arg_ptr = args + args_length; cur_arg_ptr >= args; cur_arg_ptr--) {
		save_ptr = cur_arg_ptr;
		if (cur_arg_ptr == args || (*cur_arg_ptr == ' ' && passed_arg)) {
			passed_arg = false;
			arg = strtok_r (NULL, " ", &save_ptr);
			// save_ptr is modified after strtok_r, need to point to the beginning of arg
			// save_ptr = arg;
			if (arg == NULL) continue;
			arg_size = strlen (arg) + 1;
			argc++;
			if_->rsp -= arg_size;
			memcpy (if_->rsp, arg, arg_size);
		} else if (*cur_arg_ptr != ' ') {
			passed_arg = true;
		}
	}
	// push argv[0] file name
	size_t file_name_size = strlen (file_name) + 1;
	if_->rsp -= file_name_size;
	memcpy (if_->rsp, file_name, file_name_size);

	uintptr_t start_of_args = if_->rsp; // mark the beginning of argv

	// word align
	while (if_->rsp % 8 != 0)
		if_->rsp--;
	
	if_->rsp -= 8; // argv[argc] is a null pointer

	for (uintptr_t cur = end_of_args - 2; cur >= start_of_args - 1; cur--) {
		// *(end_of_args - 1) == '\0' is null terminator of argv[argc-1]
		if (cur + 1 == start_of_args || *((char*) cur) == '\0') {
			if_->rsp -= 8;
			*((uint64_t*) if_->rsp) = cur + 1;
		}
	}

	if_->R.rsi = if_->rsp;
	if_->R.rdi = argc;
	
	if_->rsp -= 8; // fake return address
	// hex_dump(if_->rsp, if_->rsp , USER_STACK - if_->rsp , true); // for debugging

	success = true;

done:
	/* We arrive here whether the load is successful or not. */
	return success;
}


/* Checks whether PHDR describes a valid, loadable segment in
 * FILE and returns true if so, false otherwise. */
static bool
validate_segment (const struct Phdr *phdr, struct file *file) {
	/* p_offset and p_vaddr must have the same page offset. */
	if ((phdr->p_offset & PGMASK) != (phdr->p_vaddr & PGMASK))
		return false;

	/* p_offset must point within FILE. */
	if (phdr->p_offset > (uint64_t) file_length (file))
		return false;

	/* p_memsz must be at least as big as p_filesz. */
	if (phdr->p_memsz < phdr->p_filesz)
		return false;

	/* The segment must not be empty. */
	if (phdr->p_memsz == 0)
		return false;

	/* The virtual memory region must both start and end within the
	   user address space range. */
	if (!is_user_vaddr ((void *) phdr->p_vaddr))
		return false;
	if (!is_user_vaddr ((void *) (phdr->p_vaddr + phdr->p_memsz)))
		return false;

	/* The region cannot "wrap around" across the kernel virtual
	   address space. */
	if (phdr->p_vaddr + phdr->p_memsz < phdr->p_vaddr)
		return false;

	/* Disallow mapping page 0.
	   Not only is it a bad idea to map page 0, but if we allowed
	   it then user code that passed a null pointer to system calls
	   could quite likely panic the kernel by way of null pointer
	   assertions in memcpy(), etc. */
	if (phdr->p_vaddr < PGSIZE)
		return false;

	/* It's okay. */
	return true;
}

#ifndef VM
/* Codes of this block will be ONLY USED DURING project 2.
 * If you want to implement the function for whole project 2, implement it
 * outside of #ifndef macro. */

/* load() helpers. */
static bool install_page (void *upage, void *kpage, bool writable);

/* Loads a segment starting at offset OFS in FILE at address
 * UPAGE.  In total, READ_BYTES + ZERO_BYTES bytes of virtual
 * memory are initialized, as follows:
 *
 * - READ_BYTES bytes at UPAGE must be read from FILE
 * starting at offset OFS.
 *
 * - ZERO_BYTES bytes at UPAGE + READ_BYTES must be zeroed.
 *
 * The pages initialized by this function must be writable by the
 * user process if WRITABLE is true, read-only otherwise.
 *
 * Return true if successful, false if a memory allocation error
 * or disk read error occurs. */
static bool
load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes, bool writable) {
	ASSERT ((read_bytes + zero_bytes) % PGSIZE == 0);
	ASSERT (pg_ofs (upage) == 0);
	ASSERT (ofs % PGSIZE == 0);

	file_seek (file, ofs);
	while (read_bytes > 0 || zero_bytes > 0) {
		/* Do calculate how to fill this page.
		 * We will read PAGE_READ_BYTES bytes from FILE
		 * and zero the final PAGE_ZERO_BYTES bytes. */
		size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
		size_t page_zero_bytes = PGSIZE - page_read_bytes;

		/* Get a page of memory. */
		uint8_t *kpage = palloc_get_page (PAL_USER);
		if (kpage == NULL)
			return false;

		/* Load this page. */
		if (file_read (file, kpage, page_read_bytes) != (int) page_read_bytes) {
			palloc_free_page (kpage);
			return false;
		}
		memset (kpage + page_read_bytes, 0, page_zero_bytes);

		/* Add the page to the process's address space. */
		if (!install_page (upage, kpage, writable)) {
			printf("fail\n");
			palloc_free_page (kpage);
			return false;
		}

		/* Advance. */
		read_bytes -= page_read_bytes;
		zero_bytes -= page_zero_bytes;
		upage += PGSIZE;
	}
	return true;
}

/* Create a minimal stack by mapping a zeroed page at the USER_STACK */
static bool
setup_stack (struct intr_frame *if_) {
	uint8_t *kpage;
	bool success = false;

	kpage = palloc_get_page (PAL_USER | PAL_ZERO);
	if (kpage != NULL) {
		success = install_page (((uint8_t *) USER_STACK) - PGSIZE, kpage, true);
		if (success)
			if_->rsp = USER_STACK;
		else
			palloc_free_page (kpage);
	}
	return success;
}

/* Adds a mapping from user virtual address UPAGE to kernel
 * virtual address KPAGE to the page table.
 * If WRITABLE is true, the user process may modify the page;
 * otherwise, it is read-only.
 * UPAGE must not already be mapped.
 * KPAGE should probably be a page obtained from the user pool
 * with palloc_get_page().
 * Returns true on success, false if UPAGE is already mapped or
 * if memory allocation fails. */
static bool
install_page (void *upage, void *kpage, bool writable) {
	struct thread *t = thread_current ();

	/* Verify that there's not already a page at that virtual
	 * address, then map our page there. */
	return (pml4_get_page (t->pml4, upage) == NULL
			&& pml4_set_page (t->pml4, upage, kpage, writable));
}
#else
/* From here, codes will be used after project 3.
 * If you want to implement the function for only project 2, implement it on the
 * upper block. */

static bool
lazy_load_segment (struct page *page, void *aux) {
	/* TODO: Load the segment from the file */
	/* TODO: This called when the first page fault occurs on address VA. */
	/* TODO: VA is available when calling this function. */
}

/* Loads a segment starting at offset OFS in FILE at address
 * UPAGE.  In total, READ_BYTES + ZERO_BYTES bytes of virtual
 * memory are initialized, as follows:
 *
 * - READ_BYTES bytes at UPAGE must be read from FILE
 * starting at offset OFS.
 *
 * - ZERO_BYTES bytes at UPAGE + READ_BYTES must be zeroed.
 *
 * The pages initialized by this function must be writable by the
 * user process if WRITABLE is true, read-only otherwise.
 *
 * Return true if successful, false if a memory allocation error
 * or disk read error occurs. */
static bool
load_segment (struct file *file, off_t ofs, uint8_t *upage,
		uint32_t read_bytes, uint32_t zero_bytes, bool writable) {
	ASSERT ((read_bytes + zero_bytes) % PGSIZE == 0);
	ASSERT (pg_ofs (upage) == 0);
	ASSERT (ofs % PGSIZE == 0);

	while (read_bytes > 0 || zero_bytes > 0) {
		/* Do calculate how to fill this page.
		 * We will read PAGE_READ_BYTES bytes from FILE
		 * and zero the final PAGE_ZERO_BYTES bytes. */
		size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
		size_t page_zero_bytes = PGSIZE - page_read_bytes;

		/* TODO: Set up aux to pass information to the lazy_load_segment. */
		void *aux = NULL;
		if (!vm_alloc_page_with_initializer (VM_ANON, upage,
					writable, lazy_load_segment, aux))
			return false;

		/* Advance. */
		read_bytes -= page_read_bytes;
		zero_bytes -= page_zero_bytes;
		upage += PGSIZE;
	}
	return true;
}

/* Create a PAGE of stack at the USER_STACK. Return true on success. */
static bool
setup_stack (struct intr_frame *if_) {
	bool success = false;
	void *stack_bottom = (void *) (((uint8_t *) USER_STACK) - PGSIZE);

	/* TODO: Map the stack on stack_bottom and claim the page immediately.
	 * TODO: If success, set the rsp accordingly.
	 * TODO: You should mark the page is stack. */
	/* TODO: Your code goes here */

	return success;
}
#endif /* VM */
