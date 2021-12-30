# flyover-re-fixed

A fixed repository for the archived repo by [retroplasma](https://github.com/SynArchive/flyover-re-fixed)

Currently works on:

* MacOS Big Sur w/ Go 1.15.15

## Next Features

* Linux Support (again)
* Windows 10 Support
* Location Earth Rotation Model Fix

## Dependencies

* Node
* Bash
* Go
* MacOS

## Installation

```bash
go get -d github.com/SynArchive/flyover-re-fixed/...

cd "$(go env GOPATH)/src/github.com/SynArchive/flyover-re-fixed"

./scripts/get_config.sh > config.json

y
```

## Exporting Location

```
go run cmd/export-obj/main.go [lat] [lon] [zoom] [tryXY] [tryH]

node scripts/center_scale_obj.js
```

You can find the output files in: `downloaded_files/obj/<latlong>/exp_model.2.obj`

## Parameter Examples

Parameter |  Description     |  Example
----------|------------------|----------
lat       |  Latitude        |  34.007603
lon       |  Longitude       |  -118.499741
zoom      |  Zoom (~ 13-20)  |  20
tryXY     |  Area scan       |  3
tryH      |  Altitude scan   |  40

```
$ go run cmd/export-obj/main.go 34.007603 -118.499741 20 3 40
Los Angeles 111026.8 33.88808567049225 -117.9332591170418
49 exported to: ./downloaded_files/obj/34.007603--118.499741-20-3-40
```

## Important (from retroplasma)

"THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE."
