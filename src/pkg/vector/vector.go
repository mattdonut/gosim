package vector

import (
	"fmt"
	"rand"
	"math"
)

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

func (v *D3) Add(o *D3) *D3 {
	return NewD3(  v.x + o.x,
						v.y + o.y, 
						v.z + o.z  )
}

func (v * D3) Sub(o *D3) *D3 {
	return NewD3( v.x - o.x,
					v.y - o.y,
					v.z - o.z	)
}

func (v *D3) Mul(s float64) *D3 {
	res := new(D3)
	res.x = v.x * s
	res.y = v.y * s
	res.z = v.z * s
	return res
}

func (v *D3) Dot(o *D3) float64 {
	return v.x*o.x + v.y+o.y + v.z+o.z
}

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

func (v *D3) String() string {
	return fmt.Sprint(v.x,",",v.y,",",v.z)
}