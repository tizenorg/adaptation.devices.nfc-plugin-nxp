#sbs-git:slp/pkgs/n/nfc-plugin-nxp nfc-plugin-nxp 0.0.1 373e1f7d458128eca88b9f4ea73d798394449e18
%define _optdir	/opt
%define _appdir	%{_optdir}/apps
%define _ugdir	%{_optdir}/ug


Name:       nfc-plugin-nxp
Summary:    NFC Plugin for NXP Solution
Version:    0.0.7
Release:    0
Group:      TO_BE/FILLED_IN
License:    TO BE FILLED IN
Source0:    %{name}-%{version}.tar.gz
BuildRequires: cmake
BuildRequires: pkgconfig(glib-2.0)
BuildRequires: pkgconfig(vconf)
BuildRequires: pkgconfig(dlog)
BuildRequires: pkgconfig(nfc-common-lib)

%description
Description: NFC Plugin for NXP Solution


%prep
%setup -q

%build
cmake . -DCMAKE_INSTALL_PREFIX=%{_prefix}

make %{?jobs:-j%jobs}


%install
rm -rf %{buildroot}
%make_install

%post 
# file owner
if [ "${USER}" = "root" ]
then
	vconftool set -t string memory/private/nfc-plugin-nxp/eeprom "" -g 6517 -i -f
else
	vconftool set -t string memory/private/nfc-plugin-nxp/eeprom "" -f
fi


%files
%manifest nfc-plugin-nxp.manifest
%defattr(-,root,root,-) 
/usr/lib/libnfc-plugin.so
