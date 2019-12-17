#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
	float x;
	float y;
	float z;
	float w;
} Quaternion;

typedef struct {
	float x;
	float y;
	float z;
} Vector3;

Quaternion quaternionMulti(Quaternion q,Quaternion p){
	return (Quaternion){
		q.x * p.w + q.w * p.x - q.z * p.y + q.y * p.z,
		q.y * p.w + q.z * p.x + q.w * p.y - q.x * p.z,
		q.z * p.w - q.y * p.x + q.x * p.y + q.w * p.z,
		q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z};
	// float x = q.x * p.w + q.w * p.x - q.z * p.y + q.y * p.z,
	// 	  y = q.y * p.w + q.z * p.x + q.w * p.y - q.x * p.z,
	// 	  z = q.z * p.w - q.y * p.x + q.x * p.y + q.w * p.z,
	// 	  w = q.w * p.w - q.x * p.x - q.y * p.y - q.z * p.z,
	// 	  absInv = 1.0 / sqrt(x * x + y * y + z * z + w * w);
	// return (Quaternion){
	// 	x * absInv,
	// 	y * absInv,
	// 	z * absInv,
	// 	w * absInv	};
}

Quaternion quaternionInverse(Quaternion q){
	return (Quaternion){
		-q.x,
		-q.y,
		-q.z,
		 q.w	};
}

Vector3 vectorRotationToLocal(Vector3 v,Quaternion q){
	return (Vector3){
		( q.x*q.x-q.y*q.y-q.z*q.z+q.w*q.w)*v.x + 2*(q.x*q.y-q.z*q.w)*v.y + 2*(q.x*q.z+q.y*q.w)*v.z,
		2*(q.x*q.y+q.z*q.w)*v.x + (-q.x*q.x+q.y*q.y-q.z*q.z+q.w*q.w)*v.y + 2*(q.y*q.z-q.x*q.w)*v.z,
		2*(q.x*q.z-q.y*q.w)*v.x + 2*(q.y*q.z+q.x*q.w)*v.y + (-q.x*q.x-q.y*q.y+q.z*q.z+q.w*q.w)*v.z
	};
}

Vector3 vectorRotationToWorld(Vector3 v,Quaternion q){
	return (Vector3){
	 	( q.x*q.x-q.y*q.y-q.z*q.z+q.w*q.w)*v.x + 2*(q.x*q.y+q.z*q.w)*v.y + 2*(q.x*q.z-q.y*q.w)*v.z,
	 	2*(q.x*q.y-q.z*q.w)*v.x + (-q.x*q.x+q.y*q.y-q.z*q.z+q.w*q.w)*v.y + 2*(q.y*q.z+q.x*q.w)*v.z,
	 	2*(q.x*q.z+q.y*q.w)*v.x + 2*(q.y*q.z-q.x*q.w)*v.y + (-q.x*q.x-q.y*q.y+q.z*q.z+q.w*q.w)*v.z
	};
	// Quaternion temp = quaternionMulti(quaternionMulti(q,(Quaternion){v.x, v.y, v.z, 0}),quaternionInverse(q));
	// return (Vector3){temp.x, temp.y, temp.z};
}

float innerProduct(Vector3 v1,Vector3 v2){
	return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
}

Vector3 outerProduct(Vector3 v1, Vector3 v2){
	return (Vector3){
		v1.y*v2.z - v1.z*v2.y,
		v1.z*v2.x - v1.x*v2.z,
		v1.x*v2.y - v1.y*v2.x
	};
}

float vectorAbsolute(Vector3 v){
	return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
}

void Serial_print(Quaternion q){
	Serial.print(q.x,5);
	Serial.print(' ');
	Serial.print(q.y,5);
	Serial.print(' ');
	Serial.print(q.z,5);
	Serial.print(' ');
	Serial.print(q.w,5);
}

void Serial_println(Quaternion q){
	Serial.print(q.x,5);
	Serial.print(' ');
	Serial.print(q.y,5);
	Serial.print(' ');
	Serial.print(q.z,5);
	Serial.print(' ');
	Serial.println(q.w,5);
}

void Serial_print(Vector3 v){
	Serial.print(v.x,5);
	Serial.print(' ');
	Serial.print(v.y,5);
	Serial.print(' ');
	Serial.print(v.z,5);
}

void Serial_println(Vector3 v){
	Serial.print(v.x,5);
	Serial.print(' ');
	Serial.print(v.y,5);
	Serial.print(' ');
	Serial.println(v.z,5);
}
#endif