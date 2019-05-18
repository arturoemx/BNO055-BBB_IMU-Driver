/*
  RingBuffer.h  - header file of the implementation of a Ringbuffer.

    Copyright (C) 2010  Arturo Espinosa-Romero (arturoemx@gmail.com)
    Facultad de Matemáticas, Universidad Autónoma de Yucatán, México.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef __RINGBUFFER__
#define __RINGBUFFER__

#include "RingCounter.h"
#include <pthread.h>
#include <stdint.h>

template < typename X >
class RingBuffer
{
	protected:
		uint32_t RBF_Size; //The size of the RingBuffer;
		X *R;
		RingCounter H, T;//Incrementa T cuando se inserta un elemento a la cola
		//Incrementa H cuando se saca un elemento de la cola
		pthread_mutex_t RB_mutex;
	public:
		RingBuffer(uint32_t N)
		{
			pthread_mutex_init(&RB_mutex, NULL);//initialises the mutex referenced by mutex with attributes specified by attr.
			pthread_mutex_lock(&RB_mutex);
			RBF_Size = N;
			if (RBF_Size > 1)
			{
				R = new X[RBF_Size];//Reserva memoria dinamica para la cola
			}
			else
				R = 0;
			H.SetRingSize (RBF_Size);
			T.SetRingSize (RBF_Size);
			H = T = 0;
			pthread_mutex_unlock(&RB_mutex);
		}
		~RingBuffer()
		{
			pthread_mutex_lock(&RB_mutex);
			if (R)
				delete[] R;
			pthread_mutex_unlock(&RB_mutex);
		}
		virtual int Queue(const X &e)
		{
			pthread_mutex_lock(&RB_mutex);
			if (T + 1  != H)//Compara T.C con H.C
			{
				R[(uint32_t)T] = e;
				T++;//T indica cuantos elementos hay en la cola
				pthread_mutex_unlock(&RB_mutex);
				return 0;
			}
			pthread_mutex_unlock(&RB_mutex);
			return -1;
		}

		virtual int Dequeue(X &e)
		{
			pthread_mutex_lock(&RB_mutex);
			if (H != T)
			{
				e = R[(uint32_t)H];
				H++;
				pthread_mutex_unlock(&RB_mutex);
				return 0;
			}
			pthread_mutex_unlock(&RB_mutex);
			return -1;
		}
		virtual size_t getT()
		{
			return T.getC();
		}

		virtual int getH()
		{
			return H.getC();
		}

		virtual bool QueueIsEmpty()
		{
			if(T==H) return true;
			else return false;
		}

		virtual void Init_Elements(const X &e)
		{
			uint32_t i;
			pthread_mutex_lock(&RB_mutex);
			H = T = 0;//Asigna la C y RingSize de T a H
			for (i = 0; i < RBF_Size; ++i)
				R[i] = e;
			pthread_mutex_unlock(&RB_mutex);
		}
};

#endif
