/*
 * ring_buffer.c
 *
 *  Created on: 5 трав. 2023 р.
 *      Author: yevhen.surkov
 */

#include "ring_buffer.h"
#include <stdlib.h>
#include <string.h>

ring_buffer_err_t create_ring_buffer(ring_buffer_t *objPL_this, uint32_t u32L_size)
{
  if (u32L_size != 0 && objPL_this != NULL)
  {
    objPL_this->u32_front = -1;
    objPL_this->u32_rear = -1;
    objPL_this->u32_curent_size = 0;
    objPL_this->u32_size = u32L_size;
    if (((objPL_this->P_array = (uint8_t*) malloc(u32L_size * sizeof(uint8_t)))) == NULL)
    {
      return e_ring_buffer_err_allocation_error;
    }
      return e_ring_buffer_err_ok;
  }

  return e_ring_buffer_err_invalid_argument;
}

ring_buffer_err_t en_ring_buffer(ring_buffer_t *objPL_this, uint8_t *PL_item)
{
  if (PL_item != NULL && objPL_this != NULL)
  {
    if ((objPL_this->u32_front == 0 && objPL_this->u32_rear == objPL_this->u32_size - 1)
         || (objPL_this->u32_rear == (objPL_this->u32_front - 1) % (objPL_this->u32_size - 1)))
    {
      return e_ring_buffer_err_que_is_full;
    }
    else if (objPL_this->u32_front == -1) /* Insert First Element */
    {
      objPL_this->u32_front = 0;
      objPL_this->u32_rear = 0;
      memcpy(&objPL_this->P_array[objPL_this->u32_rear], PL_item, sizeof(uint8_t));
      objPL_this->u32_curent_size++;
    }
    else if (objPL_this->u32_rear == objPL_this->u32_size - 1 && objPL_this->u32_front != 0)
    {
      objPL_this->u32_rear = 0;
      memcpy(&objPL_this->P_array[objPL_this->u32_rear], PL_item, sizeof(uint8_t));
      objPL_this->u32_curent_size++;
    }
    else
    {
      objPL_this->u32_rear++;
      memcpy(&objPL_this->P_array[objPL_this->u32_rear], PL_item, sizeof(uint8_t));
      objPL_this->u32_curent_size++;
    }

    return e_ring_buffer_err_ok;
  }

  return e_ring_buffer_err_invalid_argument;
}

ring_buffer_err_t de_ring_buffer(ring_buffer_t *objPL_this, uint8_t *PL_item)
{
  if (objPL_this != NULL)
  {
    if (objPL_this->u32_curent_size == 0)
    {
      PL_item = NULL;
      return e_ring_buffer_err_que_is_empty;
    }

    memcpy(PL_item, &objPL_this->P_array[objPL_this->u32_front], sizeof(uint8_t));
    objPL_this->u32_curent_size--;
    if (objPL_this->u32_front == objPL_this->u32_rear)
    {
      objPL_this->u32_curent_size = 0;
    }
    else if (objPL_this->u32_front == objPL_this->u32_size - 1)
    {
      objPL_this->u32_front = 0;
    }
    else
    {
      objPL_this->u32_front++;
    }

    return e_ring_buffer_err_ok;
  }

  return e_ring_buffer_err_invalid_argument;
}

ring_buffer_err_t delete_ring_buffer(ring_buffer_t *objPL_this)
{
  if (objPL_this != NULL)
  {
    free(objPL_this->P_array);
    return e_ring_buffer_err_ok;
  }

  return e_ring_buffer_err_invalid_argument;
}

uint32_t get_curent_size_ring_buffer(ring_buffer_t *objPL_this)
{
  if (objPL_this != NULL)
  {
    return objPL_this->u32_curent_size;
  }

  return e_ring_buffer_err_invalid_argument;
}

