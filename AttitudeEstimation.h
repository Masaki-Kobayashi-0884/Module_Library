#include "Quaternion.h"
#include "math.h"

const float rad_per_deg = PI / 180;

Quaternion integralAngularVelocity(Quaternion q,Vector3 w,double dt){
	//q is current attitude in Quaternion
	//w is AngularVelocity in (deg/s)
	//dt is delta time in seconds
	float sx = sinf(w.x * rad_per_deg * 0.5f * dt)
	 	, sy = sinf(w.y * rad_per_deg * 0.5f * dt)
	 	, sz = sinf(w.z * rad_per_deg * 0.5f * dt)
	 	, cx = cosf(w.x * rad_per_deg * 0.5f * dt)
	 	, cy = cosf(w.y * rad_per_deg * 0.5f * dt)
	 	, cz = cosf(w.z * rad_per_deg * 0.5f * dt);
	return quaternionMulti(
		(Quaternion){	sx*cy*cz + cx*sy*sz,
						cx*sy*cz - sx*cy*sz,
						sx*sy*cz + cx*cy*sz,
					   -sx*sy*sz - cx*cy*cz 	},q);
	//return quaternionMulti(quaternionMulti(quaternionMulti((Quaternion){sx, 0, 0, cx}, (Quaternion){0, sy, 0, cy}), (Quaternion){0, 0, sz, cz}),q);
}

Quaternion generateCorrectionQuaternion(Vector3 world_v,Vector3 goal_v,Quaternion current_q,float alpha){
		Vector3 outerP = outerProduct(world_v, goal_v);
		float absolute_outerP = vectorAbsolute(outerP);
		Vector3 e = {
			outerP.x / absolute_outerP,
			outerP.y / absolute_outerP,
			outerP.z / absolute_outerP };
		float theata = acosf(innerProduct(world_v, goal_v) / (vectorAbsolute(world_v) * vectorAbsolute(goal_v)))  * 0.5f * alpha;
		float Sin = sinf(theata);
		return (Quaternion){
			e.x * Sin,
			e.y * Sin,
			e.z * Sin,
			cosf(theata)
		};
}