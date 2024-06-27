/*******************************************************************************
* Copyright (C) 2018 Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************
*/

#include "queue.h"
/*
Please note that the queue functions are not thread safe.
*/
void queue_print_all_members(queue_t *q)
{
	if (q->name){
		pr_info("==> ( %s ) - base: %p, wr: %p, rd: %p,\n"
		"\tfifo_capacity: %d, \n\tnum_item: %d, "
		"\titem_size: %d, ovf_item: %d, buffer_size: %d, max_buf_size: %d\n",
				q->name, q->base, q->wr, q->rd, q->fifo_capacity, q->num_item,
				q->item_size, q->ovf_item, q->buffer_size, q->max_buffer_size);
	}
	else
	{
		pr_info("==> base: %p, wr: %p, rd: %p, fifo_capacity: %d, num_item: %d, \n\titem_size: %d, "
			"\tovf_item: %d, buffer_size: %d, max_buf_size: %d\n",
			q->base, q->wr, q->rd, q->fifo_capacity, q->num_item, q->item_size,
			q->ovf_item, q->buffer_size, q->max_buffer_size);
	}

}

#if defined(QUEUE_DEBUG)
void queue_debug(queue_t *q)
{
	uint32_t ovf_item, num_item;
	char *name;

	enter_critical_section();
	ovf_item = q->ovf_item;
	num_item = q->num_item;
	name = q->name;
	exit_critical_section();

	if (ovf_item) {
		if (name) {
			pr_info("( %s ) - ovf_item: %d, avail:%d, q->fifo_capacity: %d\n",
				name, ovf_item, num_item, q->fifo_capacity);
		} else {
			pr_info("Queue ovf_item: %d, avail:%d, q->fifo_capacity: %d\n",
				ovf_item, num_item, q->fifo_capacity);
		}
	}
}
#else
#define queue_debug(q)
#endif

int queue_reset(queue_t *q)
{
	if (!q)
		return -EINVAL;

	enter_critical_section();
	q->wr = q->base;
	q->rd = q->base;
	q->num_item = 0;
	q->ovf_item = 0;
	q->fifo_capacity = q->buffer_size / q->item_size;
#ifdef QUEUE_USAGE_STATS
	q->pop_cnt = 0;
	q->push_cnt = 0;
	q->stats_period_cnt = 100; // Default
#endif
	exit_critical_section();
	return 0;
}

int queue_update_items_size(queue_t *q, uint32_t item_size)
{
	if (!q || !item_size || item_size > q->max_buffer_size)
		return -EINVAL;

	enter_critical_section();
	queue_reset(q);
	q->item_size = item_size;
	q->buffer_size = q->max_buffer_size - (q->max_buffer_size % item_size);
	q->fifo_capacity = q->buffer_size / q->item_size;
	exit_critical_section();
	return 0;
}

int queue_len(queue_t *q)
{
	int num_elements;

	if (!q)
		return -EINVAL;

	enter_critical_section();
	num_elements = q->num_item;
	exit_critical_section();

	return num_elements;
}

int queue_capacity(queue_t *q)
{
	int capacity;

	if (!q)
		return -EINVAL;

	enter_critical_section();
	capacity = q->fifo_capacity;
	exit_critical_section();

	return capacity;
}

int queue_buffer_size(queue_t *q)
{
	if (!q)
		return -EINVAL;
	return q->buffer_size;
}

int queue_init(queue_t *q, void *buf, uint32_t item_size, uint32_t buffer_size)
{
	if (!q || !buf || !buffer_size || !item_size || buffer_size < item_size)
		return -EINVAL;

	if (buffer_size % item_size != 0)
		return -EINVAL; // Padding problem

	enter_critical_section();
	q->num_item = 0;
	q->ovf_item = 0;
	q->base = (char *)buf;
	q->wr = (char *)buf;
	q->rd = (char *)buf;
	q->item_size = item_size;
	q->buffer_size = buffer_size;
	q->max_buffer_size = buffer_size;
	q->name = NULL;
	q->fifo_capacity = q->buffer_size / q->item_size;
#ifdef QUEUE_USAGE_STATS
	q->pop_cnt = 0;
	q->push_cnt = 0;
	q->stats_period_cnt = 100; // Default
#endif
	exit_critical_section();
	return 0;
}

int queue_init_by_name(queue_t *q, void *buf, uint32_t item_size, uint32_t buffer_size, const char *name)
{
	int ret = queue_init(q, buf, item_size, buffer_size);
	if (ret < 0)
		return ret;

	q->name = (char *)name;
	return 0;
}


void queue_destroy(queue_t *q)
{
	/*
	Buffer is not allocated within queue library. Do nothing.
	This is a placeholder function.
	*/
}

int enqueue(queue_t *q, void *data)
{
	int ret = 0;

	if (!q || !data)
		return -EINVAL; // Invalid input

	enter_critical_section();
	ret = (q->num_item != 0 && q->wr == q->rd) ? -2 : 0; // Is FIFO Full or Empty?
	memcpy((void *)q->wr, data, q->item_size);
	q->wr = q->wr + q->item_size;
	if (q->wr >= (q->base + q->buffer_size))
		q->wr = q->base;

	q->num_item++;
	if (q->num_item > q->fifo_capacity) {
		q->num_item = q->fifo_capacity;
		q->ovf_item++;
		q->rd = q->wr;
	}

	exit_critical_section();
	return ret;
}

void queue_reset_ovf(queue_t *q)
{
	if (!q)
		return;

	enter_critical_section();
	q->ovf_item = 0;
	exit_critical_section();
}

uint32_t queue_ovf_item(queue_t *q)
{
	if (!q)
		return -EINVAL;

	uint32_t ret;
	enter_critical_section();
	ret = q->ovf_item;
	exit_critical_section();
	return ret;
}

int dequeue(queue_t *q, void *data)
{
	if (!q || !data)
		return -EINVAL;

	enter_critical_section();
	if (q->num_item <= 0) {
		exit_critical_section();
		return -2;
	}

	memcpy(data, (void *)q->rd, q->item_size);
	q->num_item--;
	q->rd = q->rd + q->item_size;
	if (q->rd >= (q->base + q->buffer_size))
		q->rd = q->base;

#ifdef QUEUE_USAGE_STATS
	q->pop_cnt++;
	if ((q->pop_cnt % q->stats_period_cnt) == 0) {
		if (q->name)
			pr_info("%s:%d (%s) - %d samples lost, avail:%d \n",
				__func__, __LINE__, q->name, q->ovf_item, q->num_item);
		else
			pr_info("%s:%d - %d samples lost, avail:%d \n",
				__func__, __LINE__, q->ovf_item, q->num_item);
	}
#endif
	exit_critical_section();
	queue_debug(q);
	return 0;
}

bool queue_is_full(queue_t *q)
{
	if (!q)
		return -EINVAL;

	enter_critical_section();
	bool ret = q->wr == q->rd && q->num_item;
	exit_critical_section();
	return ret;
}

int queue_usage(queue_t *q, uint32_t *total, uint32_t *nm_item)
{
	if (!q)
		return -EINVAL;

	enter_critical_section();
	*total = q->buffer_size / q->item_size;
	*nm_item = q->num_item;
	exit_critical_section();
	return 0;
}

int queue_pop(queue_t *q)
{
	if (!q)
		return -EINVAL;

	enter_critical_section();
	if (q->num_item <= 0) {
		exit_critical_section();
		return -2;
	}

	q->num_item--;
	q->rd = q->rd + q->item_size;
	if (q->rd >= (q->base + q->buffer_size))
		q->rd = q->base;
	exit_critical_section();
#ifdef QUEUE_USAGE_STATS
	q->push_cnt++;
	if ((q->push_cnt % q->stats_period_cnt) == 0) {
		queue_debug(q);
	}
#endif
	return 0;
}

int queue_pop_n(queue_t *q, uint32_t n)
{
	if (!q || n < 1 || n > q->fifo_capacity)
		return -EINVAL;

	queue_debug(q);
	enter_critical_section();
	if (q->num_item < n) {
		exit_critical_section();
		return -2;
	}
	uint32_t curr_rd_item_off = (uint32_t)(q->rd - q->base) / q->item_size;
	uint32_t addr_off = (curr_rd_item_off + n) % (q->buffer_size / q->item_size);
	addr_off *= q->item_size;
	q->rd = q->base + addr_off;
	q->num_item -= n;
	exit_critical_section();
	return 0;
}

int queue_front(queue_t *q, void *data)
{
	if (!q || !data)
		return -EINVAL;

	enter_critical_section();
	if (q->num_item <= 0) {
		exit_critical_section();
		return -2;
	}

	memcpy(data, (void *)q->rd, q->item_size);
	exit_critical_section();
	return 0;
}

int queue_front_n(queue_t *q, void *data, uint32_t n, uint32_t buf_sz)
{
	if (!q || !data || n < 1)
		return -EINVAL;

	enter_critical_section();
	if (n > q->fifo_capacity || buf_sz < (n * q->item_size)) {
		exit_critical_section();
		return -EINVAL;
	}

	if (q->num_item < n) {
		exit_critical_section();
		return -2;
	}


	char *dest = (char *)data;
	char *rd = q->rd;
	uint32_t rd_sz = q->item_size * n;
	uint32_t to_end = (uint32_t)(q->base + q->buffer_size - q->rd);
	if (to_end < rd_sz) {
		memcpy(dest, rd, to_end);
		rd_sz -= to_end;
		rd = q->base;
		dest += to_end;
		memcpy(dest, rd, rd_sz);
   } else {
	   memcpy(dest, rd, rd_sz);
   }
#if 0
	char *dest = (char *)data;
	size_t rd_sz = q->item_size * n;
	if ((q->rd + rd_sz) > (q->base + q->buffer_size)) {
		size_t to_end = q->buffer_size - rd_sz;
		memcpy(dest, q->rd, to_end);
		dest += to_end;
		memcpy(dest, q->base, (size_t)(q->rd - q->base));
	} else
		memcpy(dest, q->rd, rd_sz);
#endif
	exit_critical_section();
	queue_debug(q);
	return 0;
}

int enqueue_string(queue_t *q, char *data, uint32_t sz)
{
	int ret = 0;
	int buf_index;
	char *wr_ptr;

	if (!q || !data || sz <= 0)
		return -EFAULT; // Invalid parameters

	enter_critical_section();
	ret = (q->wr == q->rd && q->num_item != 0) ? -2 : 0; // Is FIFO Full or Empty?
	if (q->wr >= (q->base + q->buffer_size))
		q->wr = q->base;

	if ((q->num_item + sz) > q->buffer_size) {
		exit_critical_section();
#if defined(QUEUE_DEBUG)
		{
			char buf[128];
			int len;

			if (q->name)
				len = sprintf(buf, "\r\n**** %s - ( %s ) - Fifo is full. num_item: %d, sz: %d, buffer size: %d\r\n",
						__func__, q->name, q->num_item, sz, q->buffer_size);
			else
				len = sprintf(buf, "\r\n**** %s - Fifo is full. num_item: %d, sz: %d, buffer size: %d\r\n",
						__func__, q->num_item, sz, q->buffer_size);

			UART_Write(UART_PORT, (uint8_t*)buf, len);
		}
#endif
		return -ENOMEM;
	}

	buf_index = (uint32_t)(q->wr - q->base);
	wr_ptr = q->base;
	q->num_item += sz;
	while(sz--)
		wr_ptr[buf_index++ % q->buffer_size] = *data++;

	q->wr = q->base + buf_index % q->buffer_size;
	exit_critical_section();
	return ret;
}

int dequeue_string(queue_t *q, char *buf, uint32_t buffer_size)
{
	char *rd_ptr;
	int buf_index;
	int len;

	if (!q || !buf || buffer_size <= 0)
		return -EFAULT;

	enter_critical_section();
	if (q->num_item <= 0) {
		exit_critical_section();
		return -EPERM;
	}

	rd_ptr = (char *)q->base;
	buf_index = (uint32_t)(q->rd - q->base);
	len = q->num_item;

	while (buffer_size-- && q->num_item--) {
		char tmp = rd_ptr[buf_index % q->buffer_size];
		rd_ptr[buf_index % q->buffer_size] = 0; // Remove this later on
		buf_index++;
		*buf++ = tmp;
		if (tmp == '\0')
			break;
	}

	if (q->num_item < 0) {
		/* Data corruption in FIFO */
		q->num_item = 0;
	} else
		len -= q->num_item;

	q->rd = q->base + buf_index % q->buffer_size;
	exit_critical_section();
	return len;
}

int queue_str_len(queue_t *q)
{
	char *rd_ptr;
	int buf_index;
	int len, i;

	if (!q)
		return -EFAULT;

	enter_critical_section();
	if (q->num_item <= 0) {
		exit_critical_section();
		return 0;
	}

	rd_ptr = q->base;
	buf_index = (uint32_t)(q->rd - q->base);
	i = q->num_item;
	len = 0;

	while (i--) {
		char tmp = rd_ptr[buf_index % q->buffer_size];
		buf_index++;
		if (tmp == '\0')
			break;
		len++;
	}

	exit_critical_section();
	return len;
}

void queue_n_test(void)
{
	queue_t q;
	uint8_t buf[5];
	uint8_t peek_buf[5];
	int error;
	int i;
	error = queue_init(&q, &buf[0], 1, sizeof(buf));
	if (error)
		pr_info("queue_init error :(\r\n");

	uint8_t val = 0;
	enqueue(&q, &val);
	val = 1;
	enqueue(&q, &val);
	val = 2;
	enqueue(&q, &val);
	val = 3;
	enqueue(&q, &val);
	val = 4;
	enqueue(&q, &val);

	pr_info("enqueued 0,1,2,3,4\r\n");

	error = queue_front_n(&q, &peek_buf, 5, sizeof(peek_buf));
	if (error) {
		pr_info("queue_front_n n=5 error :(\r\n");
	} else {
		pr_info("queue_front_n n=5: ");
		for (i = 0; i < 5; i++) {
			pr_info("%d ", peek_buf[i]);
		}
		pr_info("\r\n");
	}

	error = queue_front_n(&q, &peek_buf, 6, sizeof(peek_buf));
	if (error){
		pr_info("queue_front_n n=6 error :)\r\n");
	}
	else
		pr_info("queue_front_n n=6 succeeded :(\r\n");

	error = queue_pop_n(&q, 2);
	if (error){
		pr_info("queue_pop_n n=2 error :(\r\n");
	}
	else
		pr_info("queue_pop_n n=2 succeeded :)\r\n");

	error = queue_front_n(&q, &peek_buf, 5, sizeof(peek_buf));
	if (error)
		pr_info("queue_front_n n=5 error :)\r\n");

	error = queue_front_n(&q, &peek_buf, 3, sizeof(peek_buf));
	if (error) {
		pr_info("queue_front_n n=3 error :(\r\n");
	} else {
		pr_info("queue_front_n n=3: ");
		for (i = 0; i < 3; i++) {
			pr_info("%d ", peek_buf[i]);
		}
		pr_info("\r\n");
	}

	val = 0;
	enqueue(&q, &val);
	val = 1;
	enqueue(&q, &val);

	pr_info("enqueued 0,1\r\n");

	error = queue_front_n(&q, &peek_buf, 5, sizeof(peek_buf));
	if (error) {
		pr_info("queue_front_n n=5 error :(\r\n");
	} else {
		pr_info("queue_front_n n=5: ");
		for (i = 0; i < 5; i++) {
			pr_info("%d ", peek_buf[i]);
		}
		pr_info("\r\n");
	}

	error = queue_pop_n(&q, 4);
	if (error){
		pr_info("queue_pop_n n=4 error :(\r\n");
	}
	else
		pr_info("queue_pop_n n=4 succeeded :)\r\n");

	error = queue_front_n(&q, &peek_buf, 1, sizeof(peek_buf));
	if (error) {
		pr_info("queue_front_n n=1 error :(\r\n");
	} else {
		pr_info("queue_front_n n=1: ");
		for (i = 0; i < 1; i++) {
			pr_info("%d ", peek_buf[i]);
		}
		pr_info("\r\n");
	}
}
