{ pkgs ? import <nixpkgs> { }
,
}:
import
  (pkgs.fetchzip {
    url = "https://github.com/NixOS/nixpkgs/archive/refs/tags/24.05.tar.gz";
    hash = "sha256-vboIEwIQojofItm2xGCdZCzW96U85l9nDW3ifMuAIdM=";
  })
{ }
