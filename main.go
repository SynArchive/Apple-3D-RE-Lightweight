package main

import (
	"bufio"
	"bytes"
	"crypto/aes"
	"crypto/cipher"
	"crypto/rand"
	"crypto/sha256"
	"encoding/base64"
	"encoding/binary"
	"encoding/xml"
	"errors"
	"fmt"
	"io"
	"io/ioutil"
	"math"
	"math/big"
	"net/http"
	"net/url"
	"os"
	"path"
	"runtime/debug"
	"strconv"
	"strings"
	"sync"
	"time"
)

func latLonToTileTMS(zoom int, lat, lon float64) (x, y int) {
	n := float64(pow2(zoom))
	x = int(n * ((lon + 180) / 360))
	latRad := lat / 180 * math.Pi
	y = int((math.Log(math.Tan(latRad*0.5+math.Pi/4))*1/(2*math.Pi) + 0.5) * n)
	return
}

func pow2(y int) int {
	return 1 << y
}

type AltitudeManifest struct {
	XMLName  xml.Name  `xml:"manifest"`
	Triggers []Trigger `xml:"triggers>trigger"`
}

type Trigger struct {
	XMLName xml.Name `xml:"trigger"`
	Name    string   `xml:"name,attr"`
	LatRad  float64  `xml:"latitude,attr"`
	LonRad  float64  `xml:"longitude,attr"`
	Radius  float64  `xml:"radius,attr"`
	Region  int      `xml:"region,attr"`
	Version int      `xml:"version,attr"`

	Lat float64 // converted from LatRad
	Lon float64 // converted from LonRad
}

type C3MM struct {
	Header      C3MM_Header
	FileIndex   *FileIndex
	RootIndex   *RootIndex
	DataSection DataSection
}

type C3MM_Header struct {
	Unkn6            int
	FileType         uint8
	Mult1            float32
	Mult2            float32
	CompressedSize   int
	UncompressedSize int
}

type FileIndex struct {
	Entries []int
}

type RootIndex struct {
	SmallestZ int
	Entries   []Root
}

type Root struct {
	Tile          C3MM_Tile
	Offset        int
	StructureType int
}

type DataSection struct {
	Raw []byte
}

type Octant struct {
	Bits         int16
	AltitudeHigh float32
	AltitudeLow  float32
	Next         int
}

type Context struct {
	AuthContext
	ResourceManifest
}

type AuthContext struct {
	Session
	ResourceManifest
	TokenP1
}

type TokenP1 string

type Session struct {
	ID string
}

type ResourceManifest struct {
	StyleConfig          []*ResourceManifest_StyleConfig `protobuf:"bytes,2,rep,name=style_config,json=styleConfig,proto3" json:"style_config,omitempty"`
	TokenP2              string                          `protobuf:"bytes,30,opt,name=token_p2,json=tokenP2,proto3" json:"token_p2,omitempty"`
	CacheBaseUrl         string                          `protobuf:"bytes,31,opt,name=cache_base_url,json=cacheBaseUrl,proto3" json:"cache_base_url,omitempty"`
	CacheFile            []*ResourceManifest_CacheFile   `protobuf:"bytes,72,rep,name=cache_file,json=cacheFile,proto3" json:"cache_file,omitempty"`
	CacheFile_2          []string                        `protobuf:"bytes,9,rep,name=cache_file_2,json=cacheFile2,proto3" json:"cache_file_2,omitempty"`
	XXX_NoUnkeyedLiteral struct{}                        `json:"-"`
	XXX_unrecognized     []byte                          `json:"-"`
	XXX_sizecache        int32                           `json:"-"`
}

type ResourceManifest_StyleConfig struct {
	UrlPrefix_1          string   `protobuf:"bytes,1,opt,name=url_prefix_1,json=urlPrefix1,proto3" json:"url_prefix_1,omitempty"`
	UrlPrefix_2          string   `protobuf:"bytes,9,opt,name=url_prefix_2,json=urlPrefix2,proto3" json:"url_prefix_2,omitempty"`
	StyleId              int32    `protobuf:"varint,3,opt,name=style_id,json=styleId,proto3,enum=mps.ResourceManifest_StyleConfig_StyleID" json:"style_id,omitempty"`
	XXX_NoUnkeyedLiteral struct{} `json:"-"`
	XXX_unrecognized     []byte   `json:"-"`
	XXX_sizecache        int32    `json:"-"`
}

type ResourceManifest_CacheFile struct {
	FileName             string   `protobuf:"bytes,2,opt,name=file_name,json=fileName,proto3" json:"file_name,omitempty"`
	XXX_NoUnkeyedLiteral struct{} `json:"-"`
	XXX_unrecognized     []byte   `json:"-"`
	XXX_sizecache        int32    `json:"-"`
}

type writer struct {
	file   *os.File
	writer *bufio.Writer
}

type Export interface {
	Next(c3m C3M, subPfx string) error
	Close() error
}

type C3M struct {
	Header    Header
	Materials []Material
	Meshes    []Mesh
}

type Header struct {
	Translation [3]float64
	Rotation    [9]float64
}

type Material struct {
	JPEG []byte
}

type Mesh struct {
	Vertices []Vertex
	Groups   map[int]Group
}

type Vertex struct {
	X, Y, Z, U, V float32
}

type Group struct {
	Material int
	Faces    []Face
}

type Face struct {
	A, B, C int32
}

type OBJExport struct {
	dir       string
	fnPfx     string
	vtxCount  int
	objWriter writer
	mtlWriter writer
}

func findPlace(lat, long float64) Trigger {
	minDist, minPlace := math.Inf(1), Trigger{}
	for _, v := range am.Triggers {
		dist := math.Sqrt(math.Pow(lat-v.Lat, 2) + math.Pow(long-v.Lon, 2))
		if dist <= v.Radius && dist < minDist {
			minDist, minPlace = dist, v
		}
	}
	return minPlace
}

func (e *OBJExport) Close() (err error) {
	if err = e.objWriter.done(); err != nil {
		return
	}
	if err = e.mtlWriter.done(); err != nil {
		return
	}
	return
}

func (e *OBJExport) Next(c3m C3M, subPfx string) (err error) {
	defer func() {
		if e := recover(); e != nil {
			err = errors.New(fmt.Sprintln(e, string(debug.Stack())))
		}
	}()

	dir, fnPfx := e.dir, e.fnPfx

	for i, material := range c3m.Materials {
		ioutil.WriteFile(path.Join(dir, fmt.Sprintf("%s%s_%d.jpg", fnPfx, subPfx, i)), material.JPEG, 0655)
		nxt := fmt.Sprintf(`
newmtl mtl_%s_%d
Kd 1.000 1.000 1.000
d 1.0
illum 0
map_Kd %s%s_%d.jpg
`, subPfx, i, fnPfx, subPfx, i)
		e.mtlWriter.write(nxt)
	}

	for i, mesh := range c3m.Meshes {
		e.objWriter.write(fmt.Sprintf("mtllib %smodel.mtl\n", fnPfx))
		e.objWriter.write(fmt.Sprintf("o test_%s_%d\n", subPfx, i))
		for _, vtx := range mesh.Vertices {
			x, y, z := float64(vtx.X), float64(vtx.Y), float64(vtx.Z)
			if transform {
				x, y, z =
					c3m.Header.Rotation[0]*x+c3m.Header.Rotation[1]*y+c3m.Header.Rotation[2]*z,
					c3m.Header.Rotation[3]*x+c3m.Header.Rotation[4]*y+c3m.Header.Rotation[5]*z,
					c3m.Header.Rotation[6]*x+c3m.Header.Rotation[7]*y+c3m.Header.Rotation[8]*z
				x += c3m.Header.Translation[0]
				y += c3m.Header.Translation[1]
				z += c3m.Header.Translation[2]
			}

			e.objWriter.write(fmt.Sprintln("v", x, y, z))
			e.objWriter.write(fmt.Sprintln("vt", vtx.U, vtx.V))
		}

		for i, group := range mesh.Groups {
			e.objWriter.write(fmt.Sprintf("g g_%s_%d\n", subPfx, i))
			e.objWriter.write(fmt.Sprintf("usemtl mtl_%s_%d\n", subPfx, i))
			for _, face := range group.Faces {
				a, b, c := int(face.A)+1+e.vtxCount, int(face.B)+1+e.vtxCount, int(face.C)+1+e.vtxCount
				e.objWriter.write(fmt.Sprintf("f %d/%d %d/%d %d/%d\n", a, a, b, b, c, c))
			}
		}
		e.vtxCount += len(mesh.Vertices)
	}
	return
}

func newWriter(fn string) (writer, error) {
	f, err := create(fn)
	if err != nil {
		return writer{}, err
	}
	w := bufio.NewWriter(f)
	return writer{file: f, writer: w}, nil
}

func create(fn string) (*os.File, error) {
	perm := os.O_CREATE | os.O_WRONLY | os.O_TRUNC
	return os.OpenFile(fn, perm, 0655)
}

func (w *writer) write(txt string) {
	w.writer.WriteString(txt)
}

func (w writer) done() (err error) {
	if err = w.writer.Flush(); err != nil {
		return
	}
	if err = w.file.Close(); err != nil {
		return
	}
	return
}

func newExporter(dir, fnPfx string) Export {
	objWriter, _ := newWriter(path.Join(dir, fmt.Sprintf("%smodel.obj", fnPfx)))
	mtlWriter, _ := newWriter(path.Join(dir, fmt.Sprintf("%smodel.mtl", fnPfx)))
	return &OBJExport{
		dir:       dir,
		fnPfx:     fnPfx,
		vtxCount:  0,
		objWriter: objWriter,
		mtlWriter: mtlWriter,
	}
}

func tileCountPerAxis(zoom int) int {
	return pow2(zoom)
}

func quaternionToMatrix(qx, qy, qz, qw float64) (m [9]float64) {
	m[0] = 1 - 2*qy*qy - 2*qz*qz
	m[1] = 2*qx*qy - 2*qw*qz
	m[2] = 2*qx*qz + 2*qw*qy
	m[3] = 2*qx*qy + 2*qw*qz
	m[4] = 1 - 2*qx*qx - 2*qz*qz
	m[5] = 2*qy*qz - 2*qw*qx
	m[6] = 2*qx*qz - 2*qw*qy
	m[7] = 2*qy*qz + 2*qw*qx
	m[8] = 1 - 2*qx*qx - 2*qy*qy
	return
}

func parseHeader(data []byte, offset *int) Header {

	qx := ReadFloat64(data, *offset+9)
	qy := ReadFloat64(data, *offset+17)
	qz := ReadFloat64(data, *offset+25)
	qw := ReadFloat64(data, *offset+33)

	x := ReadFloat64(data, *offset+41)
	y := ReadFloat64(data, *offset+49)
	z := ReadFloat64(data, *offset+57)

	m := quaternionToMatrix(qx, qy, qz, qw)

	*offset += 113

	return Header{[3]float64{x, y, z}, m}
}

func parseMaterial(data []byte, offset *int) []Material {
	*offset += 5
	numberOfItems := int(ReadInt32(data, *offset+0))
	*offset += 4

	materials := make([]Material, numberOfItems)

	for processedItems := 0; processedItems < numberOfItems; processedItems++ {
		materialType := data[*offset]

		switch materialType {
		case 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10:

			textureFormat := data[*offset+3]
			textureOffset := ReadInt32(data, *offset+4)
			textureLength := ReadInt32(data, *offset+8)
			textureLength2 := ReadInt32(data, *offset+12)

			_ = textureLength

			switch textureFormat {
			case 0:
				materials[processedItems].JPEG = data[textureOffset : textureOffset+textureLength2]
				*offset += 16
			default:
				panic(fmt.Sprintf("Unsupported textureFormat %d", textureFormat))
			}
		default:
			panic(fmt.Sprintf("materialType %d not implemented", materialType))
		}
	}

	return materials
}

func (hp HuffmanParams) createTable() HuffmanTable {

	type tree struct {
		index     int16
		unknown4  int32
		unknown8  int32
		child1    *tree
		child2    *tree
		unknown40 int8
	}
	type tree2 struct {
		xUnknown8  int32
		xUnknown40 int8
	}

	bufs := make([]*tree, hp.p3)
	if hp.p3 == 0 {
		panic("not implemented")
	}

	for i, hp1 := 0, hp.p1; i < hp.p3; i, hp1 = i+1, hp1+hp.p0 {
		var buf tree
		buf.index = int16(i)
		buf.unknown4 = int32(0xFFFFFFFF / (hp.p2 + hp1*i))
		bufs[i] = &buf
	}

	for hp3 := hp.p3; hp3 > 1; hp3-- {
		b1 := bufs[hp3-1]
		b2 := bufs[hp3-2]

		var buf1 tree
		buf1.index = int16(-1)
		buf1.unknown4 = int32(b1.unknown4 + b2.unknown4)
		buf1.child1 = b1
		buf1.child2 = b2

		for _hp3 := hp3; ; {
			b := bufs[_hp3-2]
			if buf1.unknown4 <= b.unknown4 {
				bufs[_hp3-1] = &buf1
				break
			}
			bufs[_hp3-1] = b
			_hp3--
			if _hp3-2 == -1 {
				bufs[0] = &buf1
				break
			}
		}
	}

	buf3 := make([]*tree, 20)
	buf2 := make([]*tree2, hp.p3)
	buf3[0] = bufs[0]

	for counter := 1; counter != 0; {
		tree := buf3[counter-1]
		if tree.index < 0 {
			buf3[counter-1] = tree.child1
			buf3[counter-0] = tree.child2
			counter++
		} else {
			var t2 tree2
			t2.xUnknown8 = tree.unknown8
			t2.xUnknown40 = tree.unknown40
			buf2[tree.index] = &t2
			counter--
		}
		if tree.child1 != nil {
			tree.child1.unknown8 = 2 * tree.unknown8
			tree.child1.unknown40 = tree.unknown40 + 1
		}
		if tree.child2 != nil {
			tree.child2.unknown8 = 2*tree.unknown8 + 1
			tree.child2.unknown40 = tree.unknown40 + 1
		}
	}

	buf4 := make([]int8, 0x10001)
	buf5 := make([]int16, 0x20000/2)
	var counter int16 = 1

	for buf2idx, hp3 := 0, hp.p3; hp3 != 0; buf2idx, hp3 = buf2idx+1, hp3-1 {
		b := buf2[buf2idx]
		xu40 := b.xUnknown40
		if xu40 >= 17 {
			xu40m16 := xu40 - 16
			idx5 := b.xUnknown8 >> uint(xu40m16)
			idx4 := buf5[idx5]
			if idx4 == 0 {
				buf5[idx5] = counter
				idx4 = counter
				counter++
			}
			if xu40m16 > buf4[idx4] {
				buf4[idx4] = xu40m16
			}
		}
	}

	buf4[0] = 16
	buf6 := make([][]byte, counter)
	for i, j := 16, 0; j < int(counter); i, j = int(buf4[j+1]), j+1 {
		count := 1 << uint(i)
		buf6SubBuf := make([]byte, 8*count)
		buf6[j] = buf6SubBuf
		for ctr := 0; ctr < count; ctr++ {
			WriteUInt16(buf6SubBuf, 8*ctr, 0xFFFF)
		}
	}

	for buf2idx := 0; buf2idx < hp.p3; buf2idx++ {
		b2xu4 := buf2[buf2idx].xUnknown40
		if b2xu4 > 16 {
			b2xu8mod := buf2[buf2idx].xUnknown8 >> uint(b2xu4-16)
			b5val := buf5[b2xu8mod]
			WriteInt32(buf6[0], 8*int(b2xu8mod), int32(b5val))
			WriteInt32(buf6[0], 8*int(b2xu8mod)+4, int32(-buf4[b5val]))
			b4val := buf4[b5val]
			b2xu4m16 := b2xu4 - 16
			lob := 0xff & buf2[buf2idx].xUnknown8
			subbufPtr := int8(int8(lob&((1<<uint(b2xu4m16))-1)) << uint(b4val-b2xu4m16))
			WriteInt32(buf6[b5val], 8*int(subbufPtr), int32(buf2idx))
			WriteInt32(buf6[b5val], 8*int(subbufPtr)+4, int32(buf2[buf2idx].xUnknown40))
		} else {
			subbufPtr := buf2[buf2idx].xUnknown8 << uint(16-b2xu4)
			WriteInt32(buf6[0], 8*int(subbufPtr), int32(buf2idx))
			WriteInt32(buf6[0], 8*int(subbufPtr)+4, int32(b2xu4))
		}
	}

	for i, j := 16, 0; j < int(counter); i, j = int(buf4[j+1]), j+1 {
		if i != 0 {
			buf6SubBuf := buf6[j]
			buf6SubBufVal := ReadUInt64(buf6SubBuf, 0)
			count := 1 << uint(i)
			for ctr := 1; ctr < count; ctr++ {
				if ReadUInt16(buf6SubBuf[ctr*8:], 0) == 0xFFFF {
					WriteUInt64(buf6SubBuf, ctr*8, buf6SubBufVal)
				} else {
					buf6SubBufVal = ReadUInt64(buf6SubBuf[ctr*8:], 0)
				}
			}
		}
	}

	return HuffmanTable{buf6}
}

func readHuffmanParams(data []byte, offset int) HuffmanParams {
	return HuffmanParams{
		int(ReadInt32(data, offset+0)),
		int(ReadInt32(data, offset+4)),
		int(ReadInt32(data, offset+8)),
		int(ReadInt16(data, offset+12))}
}

type HuffmanTable struct {
	data [][]byte
}

func (hp HuffmanTable) Length() int {
	return len(hp.data)
}

type HuffmanParams struct {
	p0 int
	p1 int
	p2 int
	p3 int
}

func parseMesh(data []byte, offset *int) []Mesh {
	*offset += 5
	numberOfItems := int(ReadInt32(data, *offset+0))
	*offset += 4

	meshes := make([]Mesh, numberOfItems)

	for currentItem := 0; currentItem < numberOfItems; currentItem++ {
		meshType := ReadInt8(data, *offset+0)
		unknown_1_2 := int(ReadInt8(data, *offset+1)) + int(ReadInt8(data, *offset+2))<<8

		switch meshType {
		case 2:
			offset3 := *offset + 3

			unknownA8 := ReadInt8(data, offset3+0)

			hpa := readHuffmanParams(data, offset3+1)
			ebta := hpa.createTable()

			hpb := readHuffmanParams(data, offset3+15)
			ebtb := hpb.createTable()

			gUvCount := ReadInt32(data, offset3+29+0)
			gFacesCount := ReadInt32(data, offset3+29+4)
			groupCount := ReadInt32(data, offset3+29+8)
			dataOffset := int(ReadInt32(data, offset3+29+12))

			if groupCount == 0 && unknownA8 == 6 {
				panic("??? 1")
			}
			if unknownA8 == 8 {
				panic("??? 2")
			}

			rmd := Decompress(data, dataOffset, ebta, ebtb)
			if rmd.UVCount != gUvCount || rmd.FacesCount != gFacesCount {
				panic("decompressed mesh counts != header counts")
			}

			tmpBufFst := make([]int32, rmd.UVCount)
			for ctr := 0; ctr < 3*int(rmd.FacesCount); ctr++ {
				tmpBufFst[rmd.Res5[ctr]] = rmd.Faces[ctr]
			}
			tmpBufSnd := make([]int32, rmd.UVCount)

			preIdx := 0
			off := 0
			uvCount1 := int(rmd.UVCount)
			uvCount2 := int(rmd.UVCount)
			vertices := make([]Vertex, uvCount2)

			for {
				tmpBufFstItm := tmpBufFst[off]
				uvCountMin1 := uvCount1 - 1
				if rmd.Res8[tmpBufFstItm] != 0 {
					uvCount1 = uvCountMin1
				} else {
					uvCountMin1 = preIdx
					preIdx++
				}
				tmpBufSnd[off] = int32(uvCountMin1)
				idx := uvCountMin1
				vertices[idx].X = rmd.Vertices[3*tmpBufFstItm+0]
				vertices[idx].Y = rmd.Vertices[3*tmpBufFstItm+1]
				vertices[idx].Z = rmd.Vertices[3*tmpBufFstItm+2]
				vertices[idx].U = rmd.UV[off*2+0]
				vertices[idx].V = rmd.UV[off*2+1]

				off++
				uvCount2--
				if uvCount2 == 0 {
					break
				}
			}

			for ctr := 0; ctr < 3*int(rmd.FacesCount); ctr++ {
				rmd.Res5[ctr] = tmpBufSnd[rmd.Res5[ctr]]
			}

			gm := make(map[int]int)
			for i := 0; i < len(rmd.Res6); i++ {
				gm[int(rmd.Res6[i])]++
			}

			groups := make(map[int]Group)
			for i := 0; i < len(rmd.Res6); i++ {
				e := int(rmd.Res6[i])
				if gm[e] > 0 {
					group := groups[e]
					if group.Faces == nil {
						group.Material = e
						group.Faces = make([]Face, gm[e])
						groups[e] = group
					}
					face := &group.Faces[len(group.Faces)-gm[e]]
					face.A, face.B, face.C = rmd.Res5[i*3], rmd.Res5[i*3+1], rmd.Res5[i*3+2]
					_ = face
					gm[e]--
				}
			}

			meshes[currentItem].Groups = groups
			meshes[currentItem].Vertices = vertices

			*offset += unknown_1_2
		default:
			panic(fmt.Sprintf("Unsupported meshType %d", meshType))
		}
	}
	return meshes
}

type RawMeshData struct {
	Vertices      []float32
	VerticesCount int32
	UV            []float32
	UVCount       int32
	Faces         []int32
	Res5          []int32
	Res6          []int32
	Res7          []int32
	Res8          []int32
	FacesCount    int32
}

func decompressList(outBuf []int32, length int, inBuf []byte, outNum *int, sh int) {
	readShift := 0
	inBufOff := 0
	outBufOff := 0
	var result uint64
	if length > 0 {
		var input uint64
		for {
			if readShift < sh {
				input |= uint64(ReadUInt32BE(inBuf, inBufOff)) << uint(32-readShift)
				readShift += 32
				inBufOff += 4
			}
			result = input >> uint(64-sh)
			readShift -= sh
			input <<= uint(sh)
			outBuf[outBufOff] = int32(result)
			outBufOff++
			length--
			if length == 0 {
				break
			}
		}
	}
	*outNum += 8*inBufOff - readShift
	//l.Println("decompressList result", result)
	_ = result
}

func read10MeshBufs(data []byte, dataOffset int, ebta HuffmanTable, ebtb HuffmanTable) (bufs [10][]byte) {
	off := 120
	for i := 0; i < 10; i++ {
		len1 := int(ReadUInt32(data, dataOffset+12*i))
		len2 := int(ReadUInt32(data, dataOffset+12*i+4))
		val := ReadUInt8(data, dataOffset+12*i+8)

		desc := "?"
		switch i {
		case 0:
			desc = "header"
		case 2:
			desc = "eb clers"
		case 5:
			desc = "eb other"
		case 6:
			desc = "uv"
		case 7:
			desc = "vtx"
		}

		_ = desc

		outBuf := make([]byte, len1+3)
		switch val {
		case 0:
			buf := data[dataOffset+off : dataOffset+off+int(len2)]
			copy(outBuf, buf)
		case 3:
			buf := data[dataOffset+off : dataOffset+off+int(len2)]
			hp, s := ebta, "a"
			if i == 7 {
				hp, s = ebtb, "b"
			}
			_ = s
			hp.Decode(buf, len1, len2, &outBuf)
		case 1:
			panic("Unsupported type: 1")
		default:
			panic(fmt.Sprintf("Unknown type: %d", val))
		}
		bufs[i] = outBuf

		off += len2
	}
	return
}

func (table HuffmanTable) Decode(data []byte, len1 int, len2 int, writeBuf *[]byte) {
	readBuf := make([]byte, len2+3)
	copy(readBuf, data)
	if len1 < 2 {
		return
	}

	len2mul8 := 8 * len2
	len1div2 := len1 / 2
	tblFst := table.data[0]
	readShift1 := 0
	var input1 uint64
	readBufOff := 0
	writeBufOff := 0
	for {
		if readShift1 <= 0 {
			input1 |= uint64(ReadUInt32BE(readBuf, readBufOff)) << uint(32-readShift1)
			readShift1 += 32
			readBufOff += 4
		}
		negToggle := input1 >> 63
		readShift2 := readShift1 - 1
		input2 := 2 * input1
		shiftTest := len2mul8 - (8*readBufOff - (readShift1 - 1))
		var tblFstIdx uint64
		if shiftTest > 15 {
			if readShift1 <= 16 {
				input2 |= uint64(ReadUInt32BE(readBuf, readBufOff)) << uint(33-readShift1)
				readBufOff += 4
				readShift2 = readShift1 + 31
			}
			tblFstIdx = input2 >> 48
		} else {
			if readShift1 <= int(shiftTest) {
				input2 |= uint64(ReadUInt32BE(readBuf, readBufOff)) << uint(33-readShift1)
				readBufOff += 4
				readShift2 = readShift1 + 31
			}
			tblFstIdx = uint64(input2 >> uint(64-uint8(shiftTest)) << uint(16-shiftTest))
		}
		tblFstVal := int(ReadInt8(tblFst, 8*int(tblFstIdx)+4))
		if tblFstVal <= 0 {
			tblFstValNeg := -tblFstVal
			tblIdx := ReadInt32(tblFst, 8*int(tblFstIdx))
			if readShift2 <= 15 {
				input2 |= uint64(ReadUInt32BE(readBuf, readBufOff)) << uint(32-readShift2)
				readShift2 += 32
				readBufOff += 4
			}
			readShift3 := readShift2 - 16
			input3 := input2 << 16
			if readShift2-16 < tblFstValNeg {
				input3 |= uint64(ReadUInt32BE(readBuf, readBufOff)) << uint(48-readShift2)
				readBufOff += 4
				readShift3 = readShift2 + 16
			}
			tblOthIdx := uint(input3 >> uint(64-tblFstValNeg))
			tblOth := table.data[tblIdx]
			tblOthValNeg := int(tblOth[8*tblOthIdx+4]) - 16
			if readShift3 < tblOthValNeg {
				input3 |= uint64(ReadUInt32BE(readBuf, readBufOff)) << uint(32-readShift3)
				readShift3 += 32
				readBufOff += 4
			}
			readShift1 = readShift3 - tblOthValNeg
			input1 = input3 << uint(tblOthValNeg)
			outVal := -ReadInt32(tblOth, 8*int(tblOthIdx))
			if int32(negToggle) == 0 {
				outVal = ReadInt32(tblOth, 8*int(tblOthIdx))
			}
			WriteInt16(*writeBuf, writeBufOff, int16(outVal))
		} else {
			if readShift2 < tblFstVal {
				input2 |= uint64(ReadUInt32BE(readBuf, readBufOff)) << uint(32-readShift2)
				readShift2 += 32
				readBufOff += 4
			}
			input1 = input2 << uint(tblFstVal)
			outVal := -ReadInt32(tblFst, 8*int(tblFstIdx))
			if int32(negToggle) == 0 {
				outVal = ReadInt32(tblFst, 8*int(tblFstIdx))
			}
			WriteInt16(*writeBuf, writeBufOff, int16(outVal))
			readShift1 = readShift2 - tblFstVal
		}
		writeBufOff += 2
		len1div2--

		// end
		if len1div2 == 0 {
			break
		}
	}

	// todo check unvisited branches
}

// WriteUInt64 writes a uint64 value to data at offset
func WriteUInt64(data []byte, offset int, value uint64) {
	binary.LittleEndian.PutUint64(data[offset:], value)
}

// WriteInt64 writes an int64 value to data at offset
func WriteInt64(data []byte, offset int, value int64) {
	binary.LittleEndian.PutUint64(data[offset:], uint64(value))
}

// WriteInt32 writes an int32 value to data at offset
func WriteInt32(data []byte, offset int, value int32) {
	binary.LittleEndian.PutUint32(data[offset:], uint32(value))
}

// WriteUInt32 writes a uint32 value to data at offset
func WriteUInt32(data []byte, offset int, value uint32) {
	binary.LittleEndian.PutUint32(data[offset:], value)
}

// WriteInt16 writes an int16 value to data at offset
func WriteInt16(data []byte, offset int, value int16) {
	binary.LittleEndian.PutUint16(data[offset:], uint16(value))
}

// WriteUInt16 writes a uint16 value to data at offset
func WriteUInt16(data []byte, offset int, value uint16) {
	binary.LittleEndian.PutUint16(data[offset:], value)
}

/*
 * binary readers
 */

// ReadFloat32 reads a float32 value from data at offset
func ReadFloat32(data []byte, offset int) float32 {
	return math.Float32frombits(binary.LittleEndian.Uint32(data[offset:]))
}

// ReadFloat64 reads a float64 value from data at offset
func ReadFloat64(data []byte, offset int) float64 {
	return math.Float64frombits(binary.LittleEndian.Uint64(data[offset:]))
}

// ReadUInt8 reads a uint8 value from data at offset
func ReadUInt8(data []byte, offset int) uint8 {
	return uint8(data[offset])
}

// ReadInt8 reads an int8 value from data at offset
func ReadInt8(data []byte, offset int) int8 {
	return int8(data[offset])
}

// ReadInt16 reads an int16 value from data at offset
func ReadInt16(data []byte, offset int) int16 {
	return int16(binary.LittleEndian.Uint16(data[offset:]))
}

// ReadUInt16 reads a uint16 value from data at offset
func ReadUInt16(data []byte, offset int) uint16 {
	return binary.LittleEndian.Uint16(data[offset:])
}

// ReadInt32 reads an int32 value from data at offset
func ReadInt32(data []byte, offset int) int32 {
	return int32(binary.LittleEndian.Uint32(data[offset:]))
}

// ReadUInt32 reads a uint32 value from data at offset
func ReadUInt32(data []byte, offset int) uint32 {
	return binary.LittleEndian.Uint32(data[offset:])
}

// ReadInt32BE reads a big endian int32 value from data at offset
func ReadInt32BE(data []byte, offset int) int32 {
	return int32(binary.BigEndian.Uint32(data[offset:]))
}

// ReadUInt32BE reads a big endian uint32 value from data at offset
func ReadUInt32BE(data []byte, offset int) uint32 {
	return binary.BigEndian.Uint32(data[offset:])
}

// ReadInt64 reads an int64 value from data at offset
func ReadInt64(data []byte, offset int) int64 {
	return int64(binary.LittleEndian.Uint64(data[offset:]))
}

// ReadUInt64 reads a uint64 value from data at offset
func ReadUInt64(data []byte, offset int) uint64 {
	return binary.LittleEndian.Uint64(data[offset:])
}

// ByteSwapUInt64 returns a uint64 with its bytes swapped
func ByteSwapUInt64(value uint64) uint64 {
	return value>>56&0xff | value>>48&0xff<<8 | value>>40&0xff<<16 | value>>32&0xff<<24 |
		value>>24&0xff<<32 | value>>16&0xff<<40 | value>>8&0xff<<48 | value&0xff<<56
}

// ByteSwapUInt32 returns a uint32 with its bytes swapped
func ByteSwapUInt32(value uint32) uint32 {
	return value>>24&0xff | value>>16&0xff<<8 | value>>8&0xff<<16 | value&0xff<<24
}

func Decompress(data []byte, dataOffset int, ebta HuffmanTable, ebtb HuffmanTable) RawMeshData {
	bufs := read10MeshBufs(data, dataOffset, ebta, ebtb)

	b0 := bufs[0]
	i32_0 := ReadInt32(b0, 0)
	f64_0 := ReadFloat64(b0, 4)
	f64_1 := ReadFloat64(b0, 12)
	f64_2 := ReadFloat64(b0, 20)
	f32_0 := ReadFloat32(b0, 28)
	f32_1 := ReadFloat32(b0, 32)
	f32_2 := ReadFloat32(b0, 36)
	i8_0 := ReadUInt8(b0, 40)
	res3 := ReadInt32(b0, 41)
	i32_1 := ReadInt32(b0, 45)
	i32_2 := ReadInt32(b0, 49)
	i8_1 := ReadUInt8(b0, 53)
	i32_3 := ReadInt32(b0, 54)
	i32_4 := ReadUInt32(b0, 58)

	if i32_0 < 0 || i8_0 == 0 || (i32_1|i32_2) < 0 || i8_1 == 0 || (i32_4&0x80000000) != 0 {
		panic("incorrect values in buf 0")
	}

	b5 := bufs[5]
	res9 := ReadInt32(b5, 0)
	if res9 < 0 {
		panic("incorrect values in buf 5 #1")
	}
	i32_0min32 := i32_0 - 32
	fst := i32_0min32
	snd := 32
	buf_res9vmul3mul4_a := make([]int32, res9*3)
	for i := 0; i < len(buf_res9vmul3mul4_a); i++ {
		buf_res9vmul3mul4_a[i] = -1
	}

	buf_res9vmul3mul4_b := make([]int32, res9*3)
	for i := 0; i < len(buf_res9vmul3mul4_b); i++ {
		buf_res9vmul3mul4_b[i] = -1
	}

	if i32_0min32 >= 128 {
		for {
			val_in_data5_a := ReadInt32(b5, snd/8)
			val_in_data5_b := ReadInt32(b5, snd/8+4)
			if val_in_data5_a >= 0 {
				buf_res9vmul3mul4_a[val_in_data5_a] = val_in_data5_b
			}
			if val_in_data5_b >= 0 {
				buf_res9vmul3mul4_a[val_in_data5_b] = val_in_data5_a
			}

			fst -= 64
			snd += 64

			if !(fst > 127) {
				break
			}
		}
	}

	res1 := ReadInt32(b5, snd/8)
	b5unkn32 := ReadInt32(b5, snd/8+4)

	if (res1 | b5unkn32) < 0 {
		panic("incorrect values in buf 5 #2")
	}

	bufMetaCtr, bufMeta, writeBufOff, bufCLERS := decodeCLERS(bufs[2], res9, b5unkn32, buf_res9vmul3mul4_a)
	processCLERS(bufMeta, bufMetaCtr, bufCLERS, writeBufOff, b5unkn32, res1, buf_res9vmul3mul4_a, buf_res9vmul3mul4_b)
	res4 := buf_res9vmul3mul4_b

	res7 := mkRes7(res9, bufs[3], i32_1, buf_res9vmul3mul4_a)
	for i := 0; i < len(res7); i++ {
		if res7[i] != 0 {
			break
		}
	}

	res6 := make([]int32, res9)
	f60 := 0
	decompressList(res6, int(i32_2), bufs[4], &f60, int(i8_1))
	if i32_2 == 1 && res9 >= 2 {
		for i := range res6[1:] {
			res6[i+1] = res6[0]
		}
	}

	d9dec := make([]int32, res1)
	f70p1 := 0
	decompressList(d9dec, int(i32_3), bufs[9], &f70p1, 1)
	bufoth_a := make([]int32, res1)
	bufoth_b := make([]int32, res1)
	for i := 0; i < 3*int(res9); i++ {
		if buf_res9vmul3mul4_a[i] == -1 {
			bufoth_b[res4[align3(i)+i+2-align3(i+2)]] = 1
			bufoth_b[res4[align3(i)+i+1-align3(i+1)]] = 1
		}
	}
	for i, read := 0, 0; i < int(res1); i++ {
		if bufoth_b[i] != 0 {
			bufoth_a[i] = d9dec[read]
			read++
		}
	}
	d9dec = bufoth_a
	res8 := d9dec

	d8dec := make([]int32, res1)
	f70 := 0
	decompressList(d8dec, int(res1), bufs[8], &f70, 1)
	d1dec := make([]int32, i32_4)
	f50p1 := 0
	decompressList(d1dec, int(i32_4), bufs[1], &f50p1, 1)

	data_6_uv := make([]int16, len(bufs[6])/2)
	for i := range data_6_uv {
		data_6_uv[i] = ReadInt16(bufs[6], i*2)
	}
	data_7_vtx := make([]int16, len(bufs[7])/2)
	for i := range data_7_vtx {
		data_7_vtx[i] = ReadInt16(bufs[7], i*2)
	}

	uv_unpacked := make([]int16, res3*2)
	bf_res9mul12_a := make([]int32, res9*3)
	res5 := bf_res9mul12_a
	bf_res1mul4_a := make([]int32, res1)
	bf_res1mul4_b := make([]int32, res1)
	for i := range bf_res1mul4_b {
		bf_res1mul4_b[i] = -1
	}
	bf_res9mul4_a := make([]int32, res9)
	bf_res9mul12_b := make([]int32, res9*3)
	bf_res9mul12_c := make([]int32, res9*3)
	vtx_unpacked := make([]int16, res1*3)
	bf_res9mul4_b_res6t := make([]int32, res9)

	res3 = 0
	ctrA, ctrC, ctrB, ctrD, ctd := 0, 0, 0, 0, 0

BIG_LOOP:
	for {
		// a
		if ctrC < int(res9) {
			for {
				if bf_res9mul4_a[ctrC] == 0 {
					break
				}
				ctrC++
				if !(ctrC < int(res9)) {
					break
				}
			}
		}
		if ctrC == int(res9) {
			break
		}
		// b
		bf_res9mul12_b[ctrA] = 3 * int32(ctrC)
		ctrA++
		e1 := res4[align3(3*ctrC)+3*ctrC+2-align3(3*ctrC+2)]
		e2 := res4[3*ctrC]
		e3 := res4[align3(3*ctrC)+3*ctrC+1-align3(3*ctrC+1)]
		vtx_unpacked[e1*3+0] = data_7_vtx[e1*3+0] //e1
		vtx_unpacked[e1*3+1] = data_7_vtx[e1*3+1]
		vtx_unpacked[e1*3+2] = data_7_vtx[e1*3+2]
		vtx_unpacked[e2*3+0] = data_7_vtx[e2*3+0] //e2
		vtx_unpacked[e2*3+1] = data_7_vtx[e2*3+1]
		vtx_unpacked[e2*3+2] = data_7_vtx[e2*3+2]
		vtx_unpacked[e3*3+0] = data_7_vtx[e3*3+0] //e3
		vtx_unpacked[e3*3+1] = data_7_vtx[e3*3+1]
		vtx_unpacked[e3*3+2] = data_7_vtx[e3*3+2]
		bf_res1mul4_a[e1], bf_res1mul4_a[e2], bf_res1mul4_a[e3] = 1, 1, 1
		unpackUv(data_6_uv, uv_unpacked, &res3, res4,
			bf_res9mul12_a, bf_res1mul4_b, 3*ctrC)

		// c
		bf_res9mul4_a[ctrC] = 1
		bf_res9mul4_b_res6t[ctrC] = res6[ctrB]
		ctrB++
		if ctd|ctrA != 0 {
			ctrBplus1 := ctrB
			for {
				ctrBplus1plus0or1 := -1
				if ctrA != 0 {
					ctrBplus1plus0or1 = ctrBplus1
				} else {
					cntdwn := ctd - 1
					var v, x int32
					for {
						v = bf_res9mul12_c[cntdwn]
						x = bf_res9mul4_a[v/3]
						ctd--
						if ctd == 0 {
							break
						}
						cntdwn--
						if x == 0 {
							break
						}
					}
					if x != 0 {
						ctrA = 0
						ctrB = ctrBplus1
						continue BIG_LOOP
					}
					for i := range bf_res1mul4_b {
						bf_res1mul4_b[i] = -1
					}
					if bf_res1mul4_a[res4[v]] == 0 {
						unpackVtx(data_7_vtx, vtx_unpacked, buf_res9vmul3mul4_a, res4, bf_res1mul4_a, v)
					}
					unpackUv(data_6_uv, uv_unpacked, &res3, res4, bf_res9mul12_a, bf_res1mul4_b, int(v))
					ctrA = 1
					bf_res9mul4_a[v/3] = 1
					ctrBplus1plus0or1 = ctrBplus1 + 1
					bf_res9mul4_b_res6t[v/3] = res6[ctrBplus1]
					bf_res9mul12_b[0] = v
				}
				ctrBplus1 = ctrBplus1plus0or1
				ctrAmin1 := ctrA - 1
				cond := bf_res9mul12_b[ctrA-1]
				r6idx := ctrBplus1plus0or1 - 1
				ii := int(bf_res9mul12_b[ctrA-1])

				for {
					aVal := int(buf_res9vmul3mul4_a[ii])
					if aVal >= 0 {
						if bf_res9mul4_a[aVal/3] == 0 {
							idx1 := align3(ii) + ii + 2 - align3(ii+2)
							idx2 := align3(ii) + ii + 1 - align3(ii+1)
							other := true
							if d8dec[res4[idx1]] != 0 && d8dec[res4[idx2]] != 0 {
								ctrD++
								if d1dec[ctrD-1] != 0 {
									bf_res9mul12_c[ctd] = int32(aVal)
									ctd++
									other = false
								}
							}
							if other {
								tmp1 := res4[aVal]
								if bf_res1mul4_a[tmp1] == 0 {
									unpackVtx(data_7_vtx, vtx_unpacked,
										buf_res9vmul3mul4_a, res4,
										bf_res1mul4_a, int32(aVal))
								}
								r3tmp := bf_res1mul4_b[tmp1]
								if r3tmp == -1 {
									aValNxt := int(buf_res9vmul3mul4_a[aVal])
									nidx1 := bf_res9mul12_a[align3(aValNxt)+aValNxt+1-align3(aValNxt+1)]
									nidx2 := bf_res9mul12_a[align3(aValNxt)+aValNxt+2-align3(aValNxt+2)]
									nidx3 := bf_res9mul12_a[aValNxt]

									nu := uv_unpacked[2*nidx1+0] + uv_unpacked[2*nidx2+0] - uv_unpacked[2*nidx3+0]
									nv := uv_unpacked[2*nidx1+1] + uv_unpacked[2*nidx2+1] - uv_unpacked[2*nidx3+1]
									r3tmp = res3
									uv_unpacked[2*r3tmp] = nu - data_6_uv[2*r3tmp]
									uv_unpacked[2*r3tmp+1] = nv - data_6_uv[2*r3tmp+1]
									res3++
									bf_res1mul4_b[res4[aVal]] = r3tmp
								}
								bf_res9mul12_a[aVal] = r3tmp
								bf_res9mul12_a[align3(aVal)+aVal+2-align3(aVal+2)] = bf_res9mul12_a[idx2]
								bf_res9mul12_a[align3(aVal)+aVal+1-align3(aVal+1)] = bf_res9mul12_a[idx1]
								bf_res9mul4_a[aVal/3] = 1
								bf_res9mul4_b_res6t[aVal/3] = res6[r6idx]
								bf_res9mul12_b[ctrAmin1] = int32(aVal)
								ctrAmin1++
							}

						}
					}
					ii = align3(ii) + ii + 1 - align3(ii+1)
					if !(ii != int(cond)) {
						break
					}
				}
				ctrA = ctrAmin1

				if (ctd | ctrAmin1) == 0 {
					ctrB = ctrBplus1
					continue BIG_LOOP
				}
			}

		}
	}

	vtxBuff := make([]float32, res1*3)
	for i := 0; i < int(res1)*3; i += 3 {
		vtxBuff[i+0] = float32(float64(vtx_unpacked[i+0])*f64_0 + float64(f32_0))
		vtxBuff[i+1] = float32(float64(vtx_unpacked[i+1])*f64_1 + float64(f32_1))
		vtxBuff[i+2] = float32(float64(vtx_unpacked[i+2])*f64_2 + float64(f32_2))
	}
	res0 := vtxBuff

	uvBuff := make([]float32, res3*2)
	scale := 1.0 / float64(((uint(1) << uint(i8_0)) - 1))
	for i := 0; i < int(res3)*2; i += 2 {
		uvBuff[i+0] = float32(float64(uv_unpacked[i+0]) * scale)
		uvBuff[i+1] = float32(float64(uv_unpacked[i+1]) * scale)
	}

	res2 := uvBuff

	res6 = bf_res9mul4_b_res6t

	rmd := RawMeshData{res0, res1, res2, res3, res4, res5, res6, res7, res8, res9}

	if len(rmd.Vertices)/3 != int(rmd.VerticesCount) || len(rmd.Faces)/3 != int(rmd.FacesCount) || len(rmd.UV)/2 != int(rmd.UVCount) {
		panic("decompression error")
	}

	return rmd
}

func decodeCLERS(b2 []byte, res9 int32, b5unkn32 int32, buf_res9vmul3mul4_a []int32) (int, []int, int, []byte) {

	bufMeta := make([]int, res9)
	bufCLERS := make([]byte, res9*3)
	writeBufOff := 0
	if b5unkn32 == 0 {
		writeBufOff = 0
		if res9 > 0 {
			writeBufOff = 1
			bufCLERS[0] = 'P'
		}
	}

	if writeBufOff >= int(res9) {
		panic("not implemented: no decoding of data2")
	}
	var input uint64
	rs := 0
	bmcTmp := 0
	updown := 0
	var code uint64
	readBufOff := 0
	bufMetaCtr := bmcTmp

BIG_LOOP:
	for {
		triCtr := 3 * writeBufOff
		wboTmp := writeBufOff
		othCtr := 0
		readShift := rs
		for {
			if readShift <= 0 {
				input |= uint64(ReadUInt32BE(b2, readBufOff)) << uint(32-readShift)
				readShift += 32
				readBufOff += 4
			}
			rs = readShift - 1
			outVal := 'C'
			tmp := input & 0x8000000000000000
			flag := 0
			if tmp != 0 {
				flag = 1
			}
			input *= 2
			if flag != 0 {
				if readShift <= 2 {
					input |= uint64(ReadUInt32BE(b2, readBufOff)) << uint(33-readShift)
					readBufOff += 4
					rs = readShift + 31
				}
				code = input >> 62
				rs -= 2
				input *= 4
				if uint32(code) == 0 {
					break
				}
				if uint32(code) == 3 {
					writeBufOff += othCtr + 1
					bufCLERS[wboTmp+othCtr] = 'E'
					if updown > 0 {
						updown--
						if writeBufOff < int(res9) {
							continue BIG_LOOP
						}
						break BIG_LOOP
					}
					bmcTmp = bufMetaCtr + 1
					if writeBufOff < int(res9) {
						if bmcTmp >= int(b5unkn32) {
							bufCLERS[wboTmp+1+othCtr] = 'P'
							writeBufOff = wboTmp + othCtr + 2
						} else {
							bufMeta[bufMetaCtr+1] = writeBufOff
						}
					}
					if writeBufOff >= int(res9) {
						bufMetaCtr++
						break BIG_LOOP
					}
					bufMetaCtr = bmcTmp
					continue BIG_LOOP
				}
				outVal = 'L'
				if uint32(code) == 1 {
					outVal = 'R'
				}
			}
			bufCLERS[writeBufOff+othCtr] = byte(outVal)
			othCtr++
			triCtr += 3
			readShift = rs
			if othCtr+writeBufOff >= int(res9) {
				writeBufOff += othCtr
				break BIG_LOOP
			}
		}
		bufCLERS[writeBufOff+othCtr] = 'S'

		idx := triCtr + 2 - align3(triCtr+2) + align3(triCtr)

		if buf_res9vmul3mul4_a[idx] == -1 {
			updown++
		}
		writeBufOff += othCtr + 1

		if writeBufOff < int(res9) {
			continue
		}
		break
	}

	return bufMetaCtr, bufMeta, writeBufOff, bufCLERS
}

func processCLERS(bufMeta []int, bufMetaCtr int, bufCLERS []byte, writeBufOff int, _b5unkn32 int32, res1 int32, buf_res9vmul3mul4_a []int32, buf_res9vmul3mul4_b []int32) {
	b5unkn32 := int(_b5unkn32)
	if bufMetaCtr <= 0 {
		panic("not implemented: bufMetaCtr <= 0")
	}
	if writeBufOff <= 0 {
		panic("not implemeted: writeBufOff <= 0")
	}
	res1vaga := 0
	res1v_min1ag := res1 - 1
	tmpBuf := make([]int32, 3*writeBufOff)
	writeBufOff--

	for {
		res9vmul3agb := b5unkn32
		bufMetaCtr--
		tmp1 := -1
		for {
			if !(bufMetaCtr >= b5unkn32 || bufMetaCtr >= 0 && writeBufOff >= bufMeta[bufMetaCtr]) {
				break
			}
			clersVal := bufCLERS[writeBufOff]

			switch clersVal {
			case 'C':
				writeBufOffMul3 := 3 * writeBufOff
				if tmp1 >= 0 {
					buf_res9vmul3mul4_a[tmp1] = int32(3*writeBufOff + 1)
				}
				if writeBufOffMul3 >= -1 {
					buf_res9vmul3mul4_a[3*writeBufOff+1] = int32(tmp1)
				}
				tmp2 := res1v_min1ag
				res1v_min1ag--
				closeStar(buf_res9vmul3mul4_a, buf_res9vmul3mul4_b, writeBufOffMul3+2, tmp2)
				b5unkn32 = res9vmul3agb
			case 'L':
				if tmp1 >= 0 {
					buf_res9vmul3mul4_a[tmp1] = int32(3*writeBufOff + 1)
				}
				if 3*writeBufOff >= -1 {
					buf_res9vmul3mul4_a[3*writeBufOff+1] = int32(tmp1)
				}
			case 'E':
				if tmp1 > 0 {
					tmpBuf[res1vaga] = int32(tmp1)
					res1vaga++
				}
			case 'R':
				tmp3 := 3*writeBufOff + 2
				if tmp1 >= 0 {
					buf_res9vmul3mul4_a[tmp1] = int32(tmp3)
				}
				if tmp3 >= 0 {
					buf_res9vmul3mul4_a[3*writeBufOff+2] = int32(tmp1)
				}
			case 'S':
				writeBufOffMul3_2 := 3 * writeBufOff
				if tmp1 >= 0 {
					buf_res9vmul3mul4_a[tmp1] = int32(3*writeBufOff + 1)
				}
				if writeBufOffMul3_2 >= -1 {
					buf_res9vmul3mul4_a[3*writeBufOff+1] = int32(tmp1)
				}
				tmp4 := writeBufOffMul3_2 + 2
				tmp5 := buf_res9vmul3mul4_a[3*writeBufOff+2]
				if tmp5 == -1 {
					tmp6 := tmpBuf[res1vaga-1]
					if tmp4 >= 0 {
						buf_res9vmul3mul4_a[3*writeBufOff+2] = tmp6
					}
					res1vaga--
					if tmp6 >= 0 {
						buf_res9vmul3mul4_a[tmp6] = int32(tmp4)
					}
				} else if tmp5 <= -2 {
					tmp7 := -tmp5
					if tmp4 >= 0 {
						buf_res9vmul3mul4_a[3*writeBufOff+2] = tmp7
					}
					buf_res9vmul3mul4_a[tmp7] = int32(tmp4)
					tmp8 := 0
					for {
						tmp8 = align3(tmp4) + tmp4 + 1 - align3(tmp4+1)
						tmp4 = int(buf_res9vmul3mul4_a[tmp8])
						if !(tmp4 >= 0) {
							break
						}
					}
					readBoundary(buf_res9vmul3mul4_a, buf_res9vmul3mul4_b, tmp8, &res1v_min1ag)
				}
			case 'P':
				writeBufOffMul3_1 := 3 * writeBufOff
				if tmp1 >= 0 {
					buf_res9vmul3mul4_a[tmp1] = int32(writeBufOffMul3_1)
				}
				if writeBufOff >= 0 {
					buf_res9vmul3mul4_a[3*writeBufOff] = int32(tmp1)
				}
				closeStar(buf_res9vmul3mul4_a, buf_res9vmul3mul4_b, writeBufOffMul3_1+1, res1v_min1ag-2)
				closeStar(buf_res9vmul3mul4_a, buf_res9vmul3mul4_b, writeBufOffMul3_1+2, res1v_min1ag-1)
				closeStar(buf_res9vmul3mul4_a, buf_res9vmul3mul4_b, writeBufOffMul3_1, res1v_min1ag)
				res1v_min1ag -= 3
				bufMetaCtr--
				b5unkn32 = res9vmul3agb
			default:
				//l.Println("skipping symbol", clersVal)
				panic(fmt.Sprintf("unknown symbol %b", clersVal))
			}
			tmp1 = 3 * writeBufOff
			writeBufOff--
		}

		tmp9 := 0
		if b5unkn32 != 0 {
			readBoundary(buf_res9vmul3mul4_a, buf_res9vmul3mul4_b, 3*bufMeta[bufMetaCtr]+1, &res1v_min1ag)
			tmp9 = b5unkn32 - 1
		}
		b5unkn32 = tmp9

		if bufMetaCtr <= 0 {
			break
		}
	}
	if res1v_min1ag != -1 {
		panic("res1v_min1ag not -1")
	}
}

func mkRes7(res9 int32, buf3 []byte, i32_1 int32, buf_res9vmul3mul4_a []int32) (res7 []int32) {
	res7 = make([]int32, res9)

	field58plus1 := 0
	decompressList(res7, int(i32_1), buf3, &field58plus1, 1)
	if res9 <= 0 {
		panic("res9 <= 0 not implemented")
	}

	bufres94 := make([]int32, res9)

	bufres94ptr := 1
	res9vmin1 := res9 - 1
	var bufres94tmpVal int32
	triOff := 2
	ctr := 0
	ctr2 := 0
	ctr3 := 0
	var bufres94res int32
	for {
		if bufres94tmpVal == 0 {
			ctr3 = 3 * ctr
			other := false
			if buf_res9vmul3mul4_a[triOff-2] == -1 {
				other = true
			} else if buf_res9vmul3mul4_a[triOff-1] == -1 {
				ctr3++
				other = true
			} else {
				bufres94res = 0
				ctr3 = triOff
				if buf_res9vmul3mul4_a[triOff] == -1 {
					other = true
				}
			}
			if other {
				res7idx := ctr2
				ctr2++
				bufres94res = res7[res7idx]
				if bufres94res != 0 {
					inner := buf_res9vmul3mul4_a[align3(ctr3)+ctr3+2-align3(ctr3+2)]
					bufres94[inner/3] = bufres94res
				} else {
					bufres94res = 0
				}
			}
			bufres94[bufres94ptr-1] = bufres94res
		}
		if res9vmin1 == 0 {
			return
		}
		ctr++
		bufres94tmpVal = bufres94[bufres94ptr]
		bufres94ptr++
		res9vmin1--
		triOff += 3
	}
}

func closeStar(buf_res9vmul3mul4_a []int32, buf_res9vmul3mul4_b []int32, writeBufOffMul3plus2 int, t2 int32) {
	tmp1 := align3(writeBufOffMul3plus2)
	tmp2 := tmp1 + writeBufOffMul3plus2 + 2 - align3(writeBufOffMul3plus2+2)
	tmp3 := tmp2
	tmp4 := buf_res9vmul3mul4_a[tmp2]
	tmp5 := writeBufOffMul3plus2
	tmp6 := 0
	result := 0
outer:
	// for {
	for {
		tmp6 = tmp5 + 1
		if tmp4 < 0 {
			break
		}
		result = tmp1 + tmp6 - align3(tmp6)
		buf_res9vmul3mul4_b[result] = t2
		if tmp4 == int32(writeBufOffMul3plus2) {
			break outer
		}
		tmp5 = int(buf_res9vmul3mul4_a[tmp3])
		tmp1 = align3(tmp5)
		tmp2 = tmp1 + tmp5 + 2 - align3(tmp5+2)
		tmp3 = tmp2
		tmp4 = buf_res9vmul3mul4_a[tmp2]
	}
	result = tmp1 + tmp6 - align3(tmp6)
	buf_res9vmul3mul4_b[result] = t2
	// break
	// }
	if tmp2 >= 0 {
		buf_res9vmul3mul4_a[tmp3] = int32(writeBufOffMul3plus2)
	}
	if writeBufOffMul3plus2 >= 0 {
		buf_res9vmul3mul4_a[writeBufOffMul3plus2] = int32(tmp2)
	}
	//l.Println("closeStar result", result)
	_ = result
}

func readBoundary(buf_res9vmul3mul4_a []int32, buf_res9vmul3mul4_b []int32, someIdx int, outNum *int32) {
	tmp1 := align3(someIdx) + someIdx + 1 - align3(someIdx+1)
	result := 0
	for {
		result = align3(tmp1) + tmp1 + 1 - align3(tmp1+1)
		tmp1 = int(buf_res9vmul3mul4_a[result])
		if !(tmp1 >= 0) {
			break
		}
	}
	tmp2 := *outNum
	for {
		tmp3 := align3(result)
		buf_res9vmul3mul4_b[tmp3+result+1-align3(result+1)] = tmp2
		result = tmp3 + result + 2 - align3(result+2)
		tmp4 := int(buf_res9vmul3mul4_a[result])
		var i int32
		for i = *outNum; tmp4 >= 0; i = *outNum {
			tmp5 := align3(tmp4)
			buf_res9vmul3mul4_b[tmp5+tmp4+1-align3(tmp4+1)] = i
			result = tmp5 + tmp4 + 2 - align3(tmp4+2)
			tmp4 = int(buf_res9vmul3mul4_a[result])
		}
		tmp2 = i - 1
		*outNum = tmp2

		if !(buf_res9vmul3mul4_b[align3(result)+result+1-align3(result+1)] == -1) {
			break
		}
	}
	//l.Println("readBoundary result", result)
	_ = result
}

func align3(input int) int {
	return 3 * (input / 3)
}

func unpackVtx(data_7_vtx []int16, vtx_unpacked []int16, buf_res9vmul3mul4_a []int32,
	res4 []int32, bf_res1mul4_a []int32, aVal int32) {

	idx3 := int(buf_res9vmul3mul4_a[aVal])
	vtx_h := 3 * res4[align3(idx3)+idx3+1-align3(idx3+1)]
	vtx_i := 3 * res4[align3(idx3)+idx3+2-align3(idx3+2)]
	vtx_j := 3 * res4[idx3]
	k := res4[aVal]
	vtx_k := 3 * k
	vtx_unpacked[vtx_k+0] = vtx_unpacked[vtx_h+0] + vtx_unpacked[vtx_i+0] - vtx_unpacked[vtx_j+0] - data_7_vtx[vtx_k+0]
	vtx_unpacked[vtx_k+1] = vtx_unpacked[vtx_h+1] + vtx_unpacked[vtx_i+1] - vtx_unpacked[vtx_j+1] - data_7_vtx[vtx_k+1]
	vtx_unpacked[vtx_k+2] = vtx_unpacked[vtx_h+2] + vtx_unpacked[vtx_i+2] - vtx_unpacked[vtx_j+2] - data_7_vtx[vtx_k+2]
	bf_res1mul4_a[k] = 1
}

func unpackUv(data_6_uv []int16, uv_unpacked []int16, res3 *int32, res4 []int32,
	bf_res9mul12_a []int32, bf_res1mul4_b []int32, ctrCMul3 int) {
	for _, idx := range []int{
		align3(ctrCMul3) + ctrCMul3 + 2 - align3(ctrCMul3+2),
		ctrCMul3,
		align3(ctrCMul3) + ctrCMul3 + 1 - align3(ctrCMul3+1),
	} {
		uv_unpacked[2**res3+0] = data_6_uv[2**res3+0]
		uv_unpacked[2**res3+1] = data_6_uv[2**res3+1]
		bf_res9mul12_a[idx] = *res3
		bf_res1mul4_b[res4[idx]] = bf_res9mul12_a[idx]
		*res3++
	}
}

func parseC3M(data []byte) C3M {
	size := len(data)
	if size < 134 || data[0] != 'C' || data[1] != '3' || data[2] != 'M' || data[3] != 0x03 {
		return C3M{}
		// panic("Invalid C3M v3 data")
	}

	numberOfItems := int(data[5])
	offset := 6

	var c3m C3M

	for processedItems := 0; processedItems < numberOfItems; processedItems++ {
		switch data[offset] {
		case 0:
			c3m.Header = parseHeader(data, &offset)
		case 1:
			c3m.Materials = parseMaterial(data, &offset)
		case 2:
			c3m.Meshes = parseMesh(data, &offset)
		case 3:
			return c3m
		default:
			panic("Invalid item type")
		}
	}

	return c3m
}

const (
	RM_StyleConfig_UNKWN  int32 = 0
	RM_StyleConfig_C3MM_1 int32 = 14
	RM_StyleConfig_C3M    int32 = 15
	RM_StyleConfig_DTM_1  int32 = 16
	RM_StyleConfig_DTM_2  int32 = 17
	RM_StyleConfig_C3MM_2 int32 = 52
)

func getWithCheck(url string, check func(*http.Response) error) (data []byte, err error) {
	res, err := http.Get(url)
	if err != nil {
		return
	}
	defer res.Body.Close()

	if res.StatusCode != http.StatusOK {
		err = fmt.Errorf("http status %d", res.StatusCode)
		return
	}
	err = check(res)
	if err != nil {
		return
	}
	body, err := ioutil.ReadAll(res.Body)
	if err != nil {
		return
	}
	data = body
	return
}

func getSessionID() string {
	return "3408522895400510667857544402749559481441"
}

func getAuthUrl(url string) string {
	return authURL(url, getSessionID(), tokenP1, tokenP2)
}

func authURL(urlStr string, sid string, tokenP1 string, tokenP2 string) string {
	urlObj, _ := url.ParseRequestURI(urlStr)

	tokenP3 := GenRandStr(16, "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
	token := tokenP1 + tokenP2 + tokenP3
	ts := time.Now().Unix() + 4200
	path := urlObj.RequestURI()
	sep := map[bool]string{true: "&", false: "?"}[strings.Contains(urlStr, "?")]

	plaintext := fmt.Sprintf("%s%ssid=%s%d%s", path, sep, sid, ts, tokenP3)

	plaintextBytes := padPkcs7([]byte(plaintext))
	key := hashStr(token)
	block, _ := aes.NewCipher(key)
	mode := cipher.NewCBCEncrypter(block, make([]byte, aes.BlockSize))
	ciphertext := make([]byte, len(plaintextBytes))
	mode.CryptBlocks(ciphertext, plaintextBytes)

	ciphertextEncoded := base64.StdEncoding.EncodeToString([]byte(ciphertext))
	accessKey := fmt.Sprintf("%d_%s_%s", ts, tokenP3, ciphertextEncoded)
	accessKeyEncoded := url.QueryEscape(accessKey)
	final := fmt.Sprintf(`%s%ssid=%s&accessKey=%s`, urlStr, sep, sid, accessKeyEncoded)

	return final
}

// GenRandStr generates random string from chars with length n
func GenRandStr(n int, chars string) string {
	b := make([]byte, n)
	for i := range b {
		val, err := rand.Int(rand.Reader, big.NewInt(int64(len(chars))))
		if err != nil {
			panic(err)
		}
		b[i] = chars[val.Int64()]
	}
	return string(b)
}

func hashStr(str string) []byte {
	hash := sha256.New()
	io.WriteString(hash, str)
	return hash.Sum(nil)
}

func padPkcs7(src []byte) []byte {
	padding := aes.BlockSize - len(src)%aes.BlockSize
	padtext := bytes.Repeat([]byte{byte(padding)}, padding)
	return append(src, padtext...)
}

func getFromUrl(url string) (data []byte) {
	authURL := getAuthUrl(url)
	data, _ = getWithCheck(authURL, func(res *http.Response) (err error) {
		return
	})
	return
}

type C3MM_Tile struct {
	Z int
	Y int
	X int
	H int
}

func (t C3MM_Tile) Less(b C3MM_Tile) bool {
	a := t
	if a.Z < b.Z {
		return true
	}
	if a.Z > b.Z {
		return false
	}
	if a.Y < b.Y {
		return true
	}
	if a.Y > b.Y {
		return false
	}
	if a.X < b.X {
		return true
	}
	if a.X > b.X {
		return false
	}
	return a.H < b.H
}

func (t C3MM_Tile) ZoomedOut() C3MM_Tile {
	return C3MM_Tile{Z: t.Z - 1, Y: t.Y / 2, X: t.X / 2, H: t.H / 2}
}

func getTile(p Trigger, z, y, x, h int) C3M {
	yn := tileCountPerAxis(z) - 1 - y
	url := fmt.Sprintf("%s?style=%d&v=%d&region=%d&x=%d&y=%d&z=%d&h=%d",
		urlPrefix, RM_StyleConfig_C3M, p.Version, p.Region, x, yn, z, h)
	data := getFromUrl(url)
	return parseC3M(data)
}

var am AltitudeManifest
var transform = true
var urlPrefix = "https://gspe11-ssl.ls.apple.com/tile"
var tokenP1 = "4cjLaD4jGRwlQ9U"
var tokenP2 = "72xIzEBe0vHBmf9"

func main() {

	lat, _ := strconv.ParseFloat(os.Args[1], 64)
	long, _ := strconv.ParseFloat(os.Args[2], 64)
	zoom, _ := strconv.ParseInt(os.Args[3], 10, 32)
	tryXY, _ := strconv.ParseInt(os.Args[4], 10, 32)
	tryH, _ := strconv.ParseInt(os.Args[5], 10, 32)

	z := int(zoom)
	x, y := latLonToTileTMS(z, lat, long)

	rawAm, _ := ioutil.ReadFile("./altitude.xml")
	am = AltitudeManifest{}
	_ = xml.Unmarshal(rawAm, &am)

	for i := range am.Triggers {
		t := &am.Triggers[i]
		t.Lat = t.LatRad / math.Pi * 180
		t.Lon = t.LonRad / math.Pi * 180
	}

	p := findPlace(lat, long)

	exportDir := fmt.Sprintf("./files/obj/%f-%f-%d-%d-%d", lat, long, zoom, tryXY, tryH)
	_ = os.MkdirAll(exportDir, 0755)

	export := newExporter(exportDir, "exp_")
	defer func() {
		export.Close()
	}()

	xp := 0
	dln := 1
	sem := make(chan int, dln)
	var wg sync.WaitGroup

	// exporter for decoded tiles
	ex, exDone := make(chan C3M, dln), make(chan int)

	go func() {
		for tile := range ex {
			export.Next(tile, fmt.Sprintf("%d", xp))
			xp++
		}
		exDone <- 1
	}()

	count := 0

	// loop over area and altitude
	for dx := -tryXY; dx <= tryXY; dx++ {
		for dy := -tryXY; dy <= tryXY; dy++ {
			for h := 0; h < int(tryH); h++ {
				xn := x + int(dx)
				yn := y + int(dy)
				count += 1

				if (count-1)%40 != 0 {
					continue
				}

				// async get tile
				sem <- 1
				wg.Add(1)
				dx, dy, h := dx, dy, h
				go func() {
					defer wg.Done()
					tile := getTile(p, z, yn, xn, h)

					_ = dx
					_ = dy
					ex <- tile
					<-sem
				}()
			}
		}
	}
	wg.Wait() // wait for all tile loads to finish
	close(ex) // no more tiles sent to exporter
	<-exDone  // wait till all tiles are exported
	fmt.Println("Model of area in", p.Name, "exported to:", exportDir)
}
