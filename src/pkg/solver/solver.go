package solver

import (
	"poly"
	"vector"
)

type Solver interface {
	Init(chain *poly.Chain)
	Step() *poly.Chain
}
//something that can act on a poly chain
type Modifier interface {
	Act(input, output []*poly.Chain)
	Init(system []*poly.Chain) //modifiers are all initiallized before use, given access to the chain they should act on
}

//basic solver type
type RK4 struct {
	h,h2,h6 float64
	sys [][]*poly.Chain //an array slice that will hold four copies of the system of chains
	Mods []Modifier //modifiers to be applied during the simulation
}

func (rk *RK4) Init(chains []*poly.Chain) {
	rk.sys = make([][]*poly.Chain, 4)
	rk.sys[0] = make([]*poly.Chain,len(chains))
	rk.sys[1] = make([]*poly.Chain,len(chains))
	rk.sys[2] = make([]*poly.Chain,len(chains))
	rk.sys[3] = make([]*poly.Chain,len(chains))
	for i , chain := range chains {
		rk.sys[0][i] = chain
		rk.sys[1][i] = poly.New(chain.GetLength())
		rk.sys[2][i] = poly.New(chain.GetLength())
		rk.sys[3][i] = poly.New(chain.GetLength())	
	}

	rk.h = .002 //HARD CODED HERE, should be a config somewhere else
	rk.h2 = rk.h/2
	rk.h6 = rk.h/6
}

func (rk *RK4) Step() []*poly.Chain {
	//take a step using RK4
	for n := 0; n < 100 ; n ++ { //actually, take 100
	//clear the forces
	for j, _ := range rk.sys {
		for n, _ := range rk.sys[j] {
			for i, _ := range rk.sys[j][n].Vel {
				rk.sys[j][n].Vel[i].Zero()
			}
		}
	}
	//calculate forces at the initial point (k1)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[0], rk.sys[0])
	}
	//determine the locations for the first midpoint step
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[1][n].Loc {
			rk.sys[1][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[0][n].Vel[i].Mul(rk.h2))
		}
	}
	//calculate forces for the first midpoint (k2)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[1], rk.sys[1])
	}
	//determine the locations foe the second midpoint step
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[2][n].Loc {
			rk.sys[2][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[1][n].Vel[i].Mul(rk.h2))
		}
	}
	//calculate forces for the second midpoint (k3)
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[2], rk.sys[2])
	}
	//determine the locations foe the second midpoint step
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[3][n].Loc {
			rk.sys[3][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[2][n].Vel[i].Mul(rk.h))
		}
	}
	//calculate forces for the endpoint
	for _, mod := range rk.Mods {
		mod.Act(rk.sys[3], rk.sys[3])
	}
	//apply the transform to the primary system
	for n, _ := range rk.sys[0] {
		for i, _ := range rk.sys[0][n].Loc {
			rk.sys[0][n].Loc[i] = rk.sys[0][n].Loc[i].Add(rk.sys[0][n].Vel[i].Add( rk.sys[1][n].Vel[i].Add(rk.sys[2][n].Vel[i]).Mul(2) ).Add( rk.sys[3][n].Vel[i] ).Mul(rk.h6) )
		}
	}
	}//end multiplicity loop
	return rk.sys[0]
}

func (rk *RK4) GetSystem() [][]*poly.Chain {
	return rk.sys
}

//Modifiers! these are the forces and effects felt by the chains

//SpringForce is actually the tension along the backbone of the chain, modeled as a bunch of springs
type SpringForce struct {
	k float64
}
func NewSpringForce(k float64) Modifier {
	sf := new(SpringForce)
	sf.k = k
	return sf
}
func (mod *SpringForce) Act(in, out []*poly.Chain) {
	var f1, f2 *vector.D3
	for n, _ := range in{
		for i, r := range in[n].Loc[1:len(in[n].Loc)-1] {
			f1 = in[n].Loc[i].Sub(r)
			f2 = r.Sub(in[n].Loc[i+2])
			//out.Vel[i] = vector.NewD3(2,2,2)
			out[n].Vel[i+1] = out[n].Vel[i+1].Add(f1.Sub(f1.Normalize()).Sub(f2.Sub(f2.Normalize())).Mul(mod.k))
		}
		f1 = in[n].Loc[1].Sub(in[n].Loc[0])
		f2 = in[n].Loc[len(in[n].Loc)-2].Sub(in[n].Loc[len(in[n].Loc)-1])
		out[n].Vel[0] = out[n].Vel[0].Add(f1.Sub(f1.Normalize()).Mul(mod.k))
		out[n].Vel[len(in[n].Vel)-1] = out[n].Vel[len(in[n].Vel)-1].Add(f2.Sub(f2.Normalize()).Mul(mod.k))
	}
}
func (mod *SpringForce) Init(system []*poly.Chain) {
	return
}

//StiffForce is the stiffness of the backbone of the chain, and it should be a discrete 4th derivitive
type StiffForce struct {
	k float64
}
func NewStiffForce(k float64) Modifier{
	sf := new(StiffForce)
	sf.k = k
	return sf
}
func (mod *StiffForce) Act(in, out []*poly.Chain) {
	for n, _ := range in{
		for i, _ := range in[n].Loc[2:len(in[n].Loc)-2] {
			//note that i+2 is the index for the in.Loc array slice
			out[n].Vel[i+2] = out[n].Vel[i+2].Add(in[n].Loc[i+2].Mul(2).Sub(in[n].Loc[i+4]).Sub(in[n].Loc[i]).Mul(mod.k))
		}
		out[n].Vel[1] = out[n].Vel[1].Add(in[n].Loc[1].Sub(in[n].Loc[3]).Mul(mod.k))
		out[n].Vel[0] = out[n].Vel[0].Add(in[n].Loc[0].Sub(in[n].Loc[2]).Mul(mod.k))
		out[n].Vel[len(out[n].Vel)-2] = out[n].Vel[len(out[n].Vel)-2].Add(in[n].Loc[len(out[n].Loc)-2].Sub(in[n].Loc[len(out[n].Loc)-4]).Mul(mod.k))
		out[n].Vel[len(out[n].Vel)-1] = out[n].Vel[len(out[n].Vel)-1].Add(in[n].Loc[len(out[n].Loc)-1].Sub(in[n].Loc[len(out[n].Loc)-3]).Mul(mod.k))
	}
}
func (mod *StiffForce) Init(system []*poly.Chain) {
	return
}

//PinForce pins a particular index of a particular chain to a given location, using a stiff spring
type PinForce struct {
	k float64
	chainIndex, linkIndex int
	Loc *vector.D3
}
func NewPinForce(k float64, chainIndex int, linkIndex int, loc *vector.D3) Modifier {
	pf := new(PinForce)
	pf.k = k
	pf.chainIndex = chainIndex
	pf.linkIndex = linkIndex
	pf.Loc = loc
	return pf
}
func (mod *PinForce) Init(system []*poly.Chain) {
	return
}
func (mod *PinForce) Act(in, out []*poly.Chain) {
	out[mod.chainIndex].Vel[mod.linkIndex] = out[mod.chainIndex].Vel[mod.linkIndex].Add(mod.Loc.Sub(in[mod.chainIndex].Loc[mod.linkIndex]).Mul(mod.k))
}

//AutoPinForce automatically pins the 0th vector of all chains to the position it has when the modifier is initialized
type AutoPinForce struct {
	k float64
	Pins []*vector.D3
}
func NewAutoPinForce(k float64) Modifier{
	apf := new(AutoPinForce)
	apf.k = k
	return apf
}
func (mod *AutoPinForce) Act(in, out []*poly.Chain) {
	for n, pin := range mod.Pins {
		out[n].Vel[0] = out[n].Vel[0].Add(pin.Sub(in[n].Loc[0]).Mul(mod.k))
	}
}
func (mod *AutoPinForce) Init(system []*poly.Chain) {
	mod.Pins = make([]*vector.D3,len(system))
	for n, chain := range system {
		mod.Pins[n] = vector.NewD3(0,0,0).Copy(chain.Loc[0])
	}
}

//KinesinForce applies a force tangent to the backbone of the chain
type KinesinForce struct {
	k float64
}
func NewKinesinForce(k float64) Modifier {
	kf := new(KinesinForce)
	kf.k = k
	return kf
}
func (mod *KinesinForce) Init(system []*poly.Chain) {
	return
}
func (mod *KinesinForce) Act(in, out []*poly.Chain) {
	for n, _ := range in {
		for i, _ := range in[n].Loc[1:len(in[n].Loc)-1] {
			out[n].Vel[i+1] = out[n].Vel[i+1].Add(in[n].Loc[i].Sub(in[n].Loc[i+2]).Mul(mod.k))
		}
		out[n].Vel[0] = out[n].Vel[0].Add(in[n].Loc[0].Sub(in[n].Loc[1]).Mul(mod.k))
		out[n].Vel[len(out[n].Vel)-1] = out[n].Vel[len(out[n].Vel)-1].Add(in[n].Loc[len(in[n].Loc)-2].Sub(in[n].Loc[len(in[n].Loc)-1]).Mul(mod.k))
	}
}

//ConstForce applies a force that is uniform and constant to the entire system
type ConstForce struct {
	f *vector.D3
}
func NewConstForce(f *vector.D3) Modifier {
	cf := new(ConstForce)
	cf.f = f
	return cf
}
func (mod *ConstForce) Init(system []*poly.Chain) {
	return
}
func (mod *ConstForce) Act(in, out []*poly.Chain) {
	for n,_ := range in {
		for i,_ := range in[n].Loc {
			out[n].Vel[i] = out[n].Vel[i].Add(mod.f)
		}
	}
}

//OseenTensor applies the Oseen tensor to all of the modifiers that have been acted BEFORE it is applied, but not after
type OseenTensor struct {
	k, norm, clamp float64
	vec, dif, part *vector.D3
	temp []*poly.Chain  //a temporary chain for storing results so they can be applied after the calculations are done
}
func NewOseenTensor(k float64) Modifier {
	ot := new(OseenTensor)
	ot.k = k
	ot.clamp = 10
	ot.vec = vector.NewD3(0,0,0)
	return ot
}
func (mod *OseenTensor) Init(system []*poly.Chain) {
	mod.temp = make([]*poly.Chain, len(system))
	for n,chain := range system {
		mod.temp[n] = poly.New(chain.GetLength())
	}
}
func (mod *OseenTensor) Act(in, out []*poly.Chain) {
	//madness
	//for each monomer
	for n,_ := range in {
		for i, _ := range in[n].Loc {
			mod.vec.Zero() //blank the temp vector
			for m,_ := range in {
				for j,_ := range in[m].Loc {
					if m == n && i == j {
						//don't have a monomer effect itself, just pass the velocity through
						mod.vec = mod.vec.Add(in[n].Vel[i])
						continue
					}
					//calculate the distance between the two monomers for interaction
					mod.dif = in[m].Loc[j].Sub(in[n].Loc[i])
					mod.norm = mod.dif.Norm() //this will probably not be needed with a fine timestep
					if mod.norm == 0 {
						continue
					}
					//the actual Oseen tensor
					//this should be  k *( v_mj/|diff| + (diff.(v_mj)) diff /|diff|^3 )
					mod.part = in[m].Vel[j].Add( mod.dif.Mul(mod.dif.Dot(in[m].Vel[j]) ).Mul(1.0/(mod.norm*mod.norm)) ).Mul(mod.k/mod.norm)
					//HACK, rescale the applied velocity to not exceed the driving monomer velocity
					if force, source := mod.part.Norm(), in[m].Vel[j].Norm(); force > source {
						mod.part = mod.part.Mul(source/force)
					}
					mod.vec = mod.vec.Add( mod.part )
				}
			}
			mod.temp[n].Vel[i].Copy(mod.vec)
		}
	}
	for n,_ := range mod.temp {
		for i,_ := range mod.temp[n].Vel {
			out[n].Vel[i].Copy(mod.temp[n].Vel[i])
		}
	}
}