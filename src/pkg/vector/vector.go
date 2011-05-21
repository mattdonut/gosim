package vector

import (
	"fmt"
	"rand"
	"math"
)

//very simple vector struct, with 64 bit floats
type D3 struct {
	x,y,z float64
}
func NewD3(x,y,z float64) *D3 {
	v := new(D3)
	v.x = x
	v.y = y
	v.z = z
	return v
}

func RandomD3() *D3 {
	v := new(D3)
	v.x = rand.Float64()-.5
	v.y = rand.Float64()-.5
	v.z = rand.Float64()-.5
	return v
}

func (v *D3) Norm() float64 {
	return math.Sqrt(v.x*v.x + v.y*v.y + v.z*v.z)
}
//zero a vector in place
func (v *D3) Zero() *D3 {
	v.x = 0
	v.y = 0
	v.z = 0
	return v
}
//add two vectors, returning a pointer to the result
func (v *D3) Add(o *D3) *D3 {
	return NewD3(  v.x + o.x,
						v.y + o.y, 
						v.z + o.z  )
}
//subtract two vectors, returning a pointer to the result
func (v * D3) Sub(o *D3) *D3 {
	return NewD3( v.x - o.x,
					v.y - o.y,
					v.z - o.z	)
}
//multiply a vector by a scaler, returning a pointer to the result
func (v *D3) Mul(s float64) *D3 {
	res := new(D3)
	res.x = v.x * s
	res.y = v.y * s
	res.z = v.z * s
	return res
}
//dot product between two vectors
func (v *D3) Dot(o *D3) float64 {
	return v.x*o.x + v.y+o.y + v.z+o.z
}
//copy the contents of another vector, in place
func (v *D3) Copy(o *D3) *D3 {
	v.x = o.x
	v.y = o.y
	v.z = o.z
	return v
}
//normalize a vector, returning a pointer to the new result
func (v *D3) Normalize() *D3 {
	res := new(D3)
	norm := v.Norm()
	if norm == 0 {
		return v
	}
	res.x = v.x / norm
	res.y = v.y / norm
	res.z = v.z / norm
	return res
}
//mirror the x coordinate in place
func (v *D3) MirrorX() {
	v.x = -v.x
}
//mirror the y coordinate in place
func (v *D3) MirrorY() {
	v.y = -v.y
}
//return a comma seperated string representation of the vector
func (v *D3) String() string {
	return fmt.Sprint(v.x,",",v.y,",",v.z)
}